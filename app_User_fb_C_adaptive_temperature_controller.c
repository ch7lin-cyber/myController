/***************************************************************
Description : Adaptive temperature controller

Step 6:
[MODIFY] PID raw output is kept separate from actuator limits.
[MODIFY] Hard PWM saturation is separated from output rate limiting.
[MODIFY] Anti-windup uses hard PWM saturation only.
***************************************************************/

#include "ssm_std_define.h"
#include "app_User_fb_C_pid.h"
#include "app_User_fb_C_adaptive_temperature_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

static const APP_FB_PID_PARAMETER_T default_pid =
{
    .kp = APP_FB_PID_KP_DEFAULT,
    .ki = APP_FB_PID_KI_DEFAULT,
    .kd = APP_FB_PID_KD_DEFAULT,
    .integral_limit = APP_FB_PID_INTEGRAL_LIMIT,
    .output_limit = APP_FB_PID_OUTPUT_LIMIT,
    .kaw = APP_FB_PID_KAW_DEFAULT
};

void app_fb_temperature_controller_init(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter)
{
    if(fb == 0) return;

    app_fb_feedforward_init(&fb->ff, ff_table, ff_size);

    if(pid_parameter != 0)
        app_fb_pid_init(&fb->pid, pid_parameter);
    else
        app_fb_pid_init(&fb->pid, &default_pid);

    app_fb_d_filter_init(&fb->d_filter, APP_FB_D_FILTER_ALPHA);
    app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
    app_fb_rate_limit_init(&fb->rate_limit, APP_FB_PWM_RISE_LIMIT, APP_FB_PWM_FALL_LIMIT);
    app_fb_ff_learning_init(&fb->learning, 0);

    fb->previous_pwm = 0;
    fb->state = APP_FB_STATE_IDLE;
}

void app_fb_temperature_controller_reset(APP_FB_TEMPERATURE_CONTROLLER_T *fb)
{
    if(fb == 0) return;

    app_fb_pid_reset(&fb->pid);
    app_fb_d_filter_reset(&fb->d_filter);
    app_fb_ff_learning_reset(&fb->learning);
    app_fb_rate_limit_reset(&fb->rate_limit, 0);

    fb->previous_pwm = 0;
    fb->state = APP_FB_STATE_IDLE;
}

/* Main Controller Execute: 50Hz */
void app_fb_temperature_controller_run(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    int32_t ff_pwm;
    int32_t pid_raw;
    int32_t total_raw;
    int32_t pwm_limited;
    int32_t actual_pwm;
    int32_t error;
    int32_t d_filtered;

    if(fb == 0 || input == 0 || output == 0)
        return;

    output->pwm = 0;
    output->ff_pwm = 0;
    output->pid_output = 0;
    output->ff_offset = 0;

    /* 1. Enable Check */
    if(input->enable == APP_FB_FALSE)
    {
        fb->state = APP_FB_STATE_IDLE;
        return;
    }

    /* 2. Manual Mode */
    if(input->mode == APP_FB_MODE_MANUAL)
    {
        output->pwm = APP_FB_LIMIT(input->manual_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
        return;
    }

    /* 3. Error */
    error = input->sv - input->pv;
    output->error = error;

    /* 4. Feedforward. Keep FF independent from PID saturation. */
    ff_pwm = app_fb_feedforward_run(&fb->ff, input->sv);
    ff_pwm += app_fb_ff_learning_get_offset(&fb->learning);
    output->ff_pwm = ff_pwm;

    /* 5. Integral Separation */
    fb->pid.integral_enable = app_fb_integral_separation_run(&fb->i_sep, error);

    /* 6. Derivative Filter */
    d_filtered = app_fb_d_filter_run(&fb->d_filter, input->pv);

    /* 7. PID Raw Output. No actuator clamp in PID FB. */
    pid_raw = app_fb_pid_run(&fb->pid, input->sv, input->pv, d_filtered);
    output->pid_output = pid_raw;

    /* 8. Combine FF + PID: this is the true unsaturated command. */
    total_raw = ff_pwm + pid_raw;

    /*
     * 9. Hard actuator limit only.
     * Saturation error is preserved for anti-windup.
     */
    pwm_limited = APP_FB_LIMIT(total_raw, APP_FB_PWM_MIN, APP_FB_PWM_MAX);

    /*
     * 10. Rate limit is a slew constraint, NOT a saturation source
     * for the PID back-calculation loop.
     */
    actual_pwm = app_fb_rate_limit_run(&fb->rate_limit, pwm_limited);
    output->pwm = actual_pwm;

    /*
     * 11. Anti-Windup: hard PWM saturation only.
     * Use the hard-limited command, not the rate-limited output.
     *
     * total_raw = 1050, pwm_limited = 1000, actual_pwm may be 630.
     * Anti-windup sees 1000 - 1050 = -50, not 630 - 1050.
     */
    app_fb_pid_anti_windup(&fb->pid, total_raw, pwm_limited);

    /* 12. Adaptive Learning */
    app_fb_ff_learning_run(&fb->learning, input->sv, input->pv, pid_raw);
    output->ff_offset = app_fb_ff_learning_get_offset(&fb->learning);

    fb->state = APP_FB_STATE_RUN;
}

#ifdef __cplusplus
}
#endif
