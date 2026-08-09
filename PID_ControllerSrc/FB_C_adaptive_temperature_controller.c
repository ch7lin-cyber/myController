#include <stdint.h>
#include "FB_C_adaptive_temperature_controller.h"
#include "FB_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_controller_limit_i64(int64_t value, int32_t min_value, int32_t max_value)
{
    if(value > (int64_t)max_value) return max_value;
    if(value < (int64_t)min_value) return min_value;
    return (int32_t)value;
}

static int64_t app_fb_controller_abs_i64(int64_t value)
{
    return (value >= 0) ? value : -value;
}

MY_API void app_fb_temperature_controller_init
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter
)
{
    app_fb_temperature_controller_init_ex(
        fb,
        ff_table,
        ff_size,
        pid_parameter,
        0);
}

MY_API void app_fb_temperature_controller_init_ex
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter
)
{
    if(fb == 0) return;

    app_fb_pid_init(&fb->pid, pid_parameter);
    app_fb_feedforward_init(&fb->ff, ff_table, ff_size);
    app_fb_d_filter_init(&fb->d_filter, APP_FB_D_FILTER_ALPHA);
    app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
    app_fb_rate_limit_init(&fb->rate_limit, APP_FB_PWM_RISE_LIMIT, APP_FB_PWM_FALL_LIMIT);
    app_fb_ff_learning_init(&fb->learning, adaptive_parameter);

    fb->previous_pwm = 0;
    fb->previous_sv = 0;
    fb->manual_active = APP_FB_FALSE;
    fb->sv_initialized = APP_FB_FALSE;
    fb->integral_disturbance_armed = APP_FB_FALSE;
    fb->learning_enabled = APP_FB_TRUE;
    fb->state = APP_FB_STATE_IDLE;
}

MY_API void app_fb_temperature_controller_set_adaptive_parameter
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter
)
{
    if(fb == 0) return;
    app_fb_ff_learning_reconfigure(&fb->learning, adaptive_parameter);
}

MY_API void app_fb_temperature_controller_set_pid_parameter
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_PID_PARAMETER_T *pid_parameter
)
{
    if(fb == 0 || pid_parameter == 0) return;

    /* Candidate changes must begin from a deterministic dynamic state. */
    app_fb_pid_init(&fb->pid, pid_parameter);
    app_fb_d_filter_reset(&fb->d_filter);
    app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
    fb->integral_disturbance_armed = APP_FB_FALSE;

    /* Keep the actual output history for rate limiting so a PID update does
       not force an artificial output step. */
    app_fb_rate_limit_reset(&fb->rate_limit, fb->previous_pwm);
}

MY_API void app_fb_temperature_controller_set_learning_enabled
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    APP_FB_BOOL enabled
)
{
    if(fb == 0) return;
    fb->learning_enabled = (enabled != APP_FB_FALSE) ? APP_FB_TRUE : APP_FB_FALSE;
}

MY_API void app_fb_temperature_controller_reset
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb
)
{
    if(fb == 0) return;

    app_fb_pid_reset(&fb->pid);

    if(fb->ff.table != 0 && fb->ff.size > 0)
        app_fb_feedforward_init(&fb->ff, fb->ff.table, fb->ff.size);
    else
        fb->ff.output = 0;

    app_fb_d_filter_reset(&fb->d_filter);
    app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
    app_fb_rate_limit_reset(&fb->rate_limit, 0);
    app_fb_ff_learning_reset(&fb->learning);

    fb->previous_pwm = 0;
    fb->previous_sv = 0;
    fb->manual_active = APP_FB_FALSE;
    fb->sv_initialized = APP_FB_FALSE;
    fb->integral_disturbance_armed = APP_FB_FALSE;
    fb->learning_enabled = APP_FB_TRUE;
    fb->state = APP_FB_STATE_IDLE;
}

/* Main Controller Execute: 50Hz */
MY_API void app_fb_temperature_controller_run(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    int32_t ff_base_pwm;
    int32_t ff_offset;
    int32_t pid_raw;
    int32_t total_raw;
    int32_t pwm_limited;
    int32_t actual_pwm;
    int32_t error;
    int32_t d_filtered;
    int32_t desired_pid_output;
    int64_t desired_pid_output64;
    int64_t total_raw64;
    int64_t sv_delta;
    APP_FB_BOOL bumpless_transition;
    APP_FB_BOOL learning_allowed;

    if(fb == 0 || input == 0 || output == 0) return;

    output->pwm = 0;
    output->ff_pwm = 0;
    output->pid_output = 0;
    output->ff_offset = 0;
    output->error = 0;

    if(input->enable == APP_FB_FALSE)
    {
        app_fb_pid_reset(&fb->pid);
        app_fb_d_filter_reset(&fb->d_filter);
        app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
        app_fb_rate_limit_reset(&fb->rate_limit, 0);
        fb->previous_pwm = 0;
        fb->previous_sv = 0;
        fb->manual_active = APP_FB_FALSE;
        fb->sv_initialized = APP_FB_FALSE;
        fb->integral_disturbance_armed = APP_FB_FALSE;
        fb->state = APP_FB_STATE_IDLE;
        return;
    }

    if(input->mode == APP_FB_MODE_MANUAL)
    {
        int32_t manual_pwm_limited;
        app_fb_pid_reset(&fb->pid);
        app_fb_d_filter_reset(&fb->d_filter);
        app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
        manual_pwm_limited = APP_FB_LIMIT(input->manual_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
        app_fb_rate_limit_reset(&fb->rate_limit, manual_pwm_limited);
        output->pwm = manual_pwm_limited;
        fb->previous_pwm = output->pwm;
        fb->previous_sv = input->sv;
        fb->sv_initialized = APP_FB_TRUE;
        fb->integral_disturbance_armed = APP_FB_FALSE;
        fb->manual_active = APP_FB_TRUE;
        fb->state = APP_FB_STATE_RUN;
        return;
    }

    if(fb->sv_initialized == APP_FB_FALSE)
    {
        fb->previous_sv = input->sv;
        fb->sv_initialized = APP_FB_TRUE;
        fb->integral_disturbance_armed = APP_FB_FALSE;
    }
    else
    {
        sv_delta = (int64_t)input->sv - (int64_t)fb->previous_sv;
        if(app_fb_controller_abs_i64(sv_delta) >= (int64_t)APP_FB_I_SV_CHANGE_THRESHOLD)
        {
            fb->previous_sv = input->sv;
            fb->integral_disturbance_armed = APP_FB_FALSE;
            app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
            app_fb_pid_reset(&fb->pid);
        }
    }

    error = input->sv - input->pv;
    output->error = error;

    if(app_fb_controller_abs_i64((int64_t)error) <= (int64_t)APP_FB_I_ENABLE_ERROR)
        fb->integral_disturbance_armed = APP_FB_TRUE;

    ff_base_pwm = app_fb_feedforward_run(&fb->ff, input->sv);
    ff_offset = app_fb_ff_learning_get_offset(&fb->learning);
    output->ff_pwm = ff_base_pwm;
    output->ff_offset = ff_offset;

    d_filtered = app_fb_d_filter_run(&fb->d_filter, input->pv);
    bumpless_transition = APP_FB_FALSE;

    if(fb->manual_active == APP_FB_TRUE)
    {
        desired_pid_output64 = (int64_t)fb->previous_pwm - (int64_t)ff_base_pwm - (int64_t)ff_offset;
        desired_pid_output = app_fb_controller_limit_i64(desired_pid_output64, INT32_MIN, INT32_MAX);
        fb->pid.integral_enable = APP_FB_TRUE;
        app_fb_pid_bumpless_preload(&fb->pid, input->sv, input->pv, d_filtered, desired_pid_output);
        bumpless_transition = APP_FB_TRUE;
        fb->manual_active = APP_FB_FALSE;
    }
    else if(fb->integral_disturbance_armed == APP_FB_TRUE)
    {
        fb->pid.integral_enable = APP_FB_TRUE;
    }
    else
    {
        fb->pid.integral_enable = app_fb_integral_separation_run(&fb->i_sep, error);
    }

    pid_raw = app_fb_pid_run(&fb->pid, input->sv, input->pv, d_filtered);
    output->pid_output = pid_raw;

    total_raw64 = (int64_t)ff_base_pwm + (int64_t)ff_offset + (int64_t)pid_raw;
    total_raw = app_fb_controller_limit_i64(total_raw64, INT32_MIN, INT32_MAX);
    pwm_limited = APP_FB_LIMIT(total_raw, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
    actual_pwm = app_fb_rate_limit_run(&fb->rate_limit, pwm_limited);
    output->pwm = actual_pwm;

    app_fb_pid_anti_windup(&fb->pid, total_raw, pwm_limited);

    learning_allowed = APP_FB_FALSE;
    if(fb->learning_enabled == APP_FB_TRUE &&
       total_raw >= APP_FB_PWM_MIN &&
       total_raw <= APP_FB_PWM_MAX &&
       actual_pwm == pwm_limited &&
       bumpless_transition == APP_FB_FALSE)
    {
        learning_allowed = APP_FB_TRUE;
    }

    app_fb_ff_learning_run(&fb->learning, input->sv, input->pv, pid_raw, learning_allowed);

    output->ff_offset = app_fb_ff_learning_get_offset(&fb->learning);
    fb->previous_pwm = actual_pwm;
    fb->state = APP_FB_STATE_RUN;
}

#ifdef __cplusplus
}
#endif
