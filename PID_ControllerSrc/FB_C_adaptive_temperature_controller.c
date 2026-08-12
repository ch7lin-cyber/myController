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

static APP_FB_BOOL app_fb_controller_fast_heat_state_update(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    int32_t error)
{
#if APP_FB_FAST_HEAT_ENABLE
    if(fb == 0) return APP_FB_FALSE;

    if(fb->fast_heat_active == APP_FB_FALSE)
    {
        if(error >= APP_FB_FAST_HEAT_ENTER_ERROR)
            fb->fast_heat_active = APP_FB_TRUE;
    }
    else
    {
        if(error <= APP_FB_FAST_HEAT_EXIT_ERROR)
            fb->fast_heat_active = APP_FB_FALSE;
    }

    return fb->fast_heat_active;
#else
    (void)fb;
    (void)error;
    return APP_FB_FALSE;
#endif
}

static int32_t app_fb_controller_fast_heat_blend(int32_t error, int32_t normal_pwm)
{
#if APP_FB_FAST_HEAT_ENABLE
    int64_t span;
    int64_t x_q15;
    int64_t smooth_q15;
    int64_t scale;
    int64_t target;

    normal_pwm = APP_FB_LIMIT(normal_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);

    if(error <= APP_FB_FAST_HEAT_ENTER_ERROR)
        return normal_pwm;

    if(error >= APP_FB_FAST_HEAT_FULL_ERROR)
        return APP_FB_LIMIT(APP_FB_FAST_HEAT_FULL_PWM, APP_FB_PWM_MIN, APP_FB_PWM_MAX);

    span = (int64_t)APP_FB_FAST_HEAT_FULL_ERROR - (int64_t)APP_FB_FAST_HEAT_ENTER_ERROR;
    scale = (int64_t)APP_FB_FAST_HEAT_BLEND_SCALE;
    if(span <= 0 || scale <= 0) return normal_pwm;

    x_q15 = (((int64_t)error - (int64_t)APP_FB_FAST_HEAT_ENTER_ERROR) * scale) / span;
    if(x_q15 < 0) x_q15 = 0;
    if(x_q15 > scale) x_q15 = scale;

    smooth_q15 = (x_q15 * x_q15 * ((3 * scale) - (2 * x_q15))) / (scale * scale);
    target = (int64_t)normal_pwm +
             (smooth_q15 * ((int64_t)APP_FB_FAST_HEAT_FULL_PWM - (int64_t)normal_pwm)) / scale;

    return app_fb_controller_limit_i64(target, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
#else
    (void)error;
    return APP_FB_LIMIT(normal_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
#endif
}

static int32_t app_fb_controller_fast_heat_tail_boost(int32_t error, int32_t boosted_pwm)
{
#if APP_FB_FAST_HEAT_TAIL_ENABLE
    int64_t span;
    int64_t x_q15;
    int64_t smooth_q15;
    int64_t scale;
    int64_t add_pwm;
    int64_t target;

    boosted_pwm = APP_FB_LIMIT(boosted_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);

    if(error <= APP_FB_FAST_HEAT_TAIL_START_ERROR)
        return boosted_pwm;

    if(error >= APP_FB_FAST_HEAT_TAIL_FULL_ERROR)
        return APP_FB_LIMIT(boosted_pwm + APP_FB_FAST_HEAT_TAIL_MAX_ADD_PWM,
                            APP_FB_PWM_MIN, APP_FB_PWM_MAX);

    span = (int64_t)APP_FB_FAST_HEAT_TAIL_FULL_ERROR -
           (int64_t)APP_FB_FAST_HEAT_TAIL_START_ERROR;
    scale = (int64_t)APP_FB_FAST_HEAT_BLEND_SCALE;
    if(span <= 0 || scale <= 0) return boosted_pwm;

    x_q15 = (((int64_t)error - (int64_t)APP_FB_FAST_HEAT_TAIL_START_ERROR) * scale) / span;
    if(x_q15 < 0) x_q15 = 0;
    if(x_q15 > scale) x_q15 = scale;

#if APP_FB_FAST_HEAT_TAIL_CONCAVE_ENABLE
    /* V3.5: concave tail curve y = 1 - (1-x)^2 = 2x - x^2. */
    smooth_q15 = (2 * x_q15) - ((x_q15 * x_q15) / scale);
#else
    smooth_q15 = (x_q15 * x_q15 * ((3 * scale) - (2 * x_q15))) / (scale * scale);
#endif
    add_pwm = (smooth_q15 * (int64_t)APP_FB_FAST_HEAT_TAIL_MAX_ADD_PWM) / scale;
    target = (int64_t)boosted_pwm + add_pwm;

    return app_fb_controller_limit_i64(target, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
#else
    (void)error;
    return APP_FB_LIMIT(boosted_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
#endif
}

static int32_t app_fb_controller_predict_error(
    const APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    int32_t sv,
    int32_t pv,
    int32_t filtered_delta_pv)
{
#if APP_FB_PREDICTIVE_BRAKE_ENABLE
    int64_t predicted_delta;
    int64_t predicted_pv;
    int64_t predicted_error;

    if(fb == 0 || fb->timing.sample_time_ms == 0U)
        return sv - pv;

    /* filtered_delta_pv is temperature change per controller sample. */
    predicted_delta = ((int64_t)filtered_delta_pv *
                       (int64_t)APP_FB_PREDICTIVE_BRAKE_TIME_MS) /
                      (int64_t)fb->timing.sample_time_ms;
    predicted_pv = (int64_t)pv + predicted_delta;
    predicted_error = (int64_t)sv - predicted_pv;
    return app_fb_controller_limit_i64(predicted_error, INT32_MIN, INT32_MAX);
#else
    (void)fb;
    (void)filtered_delta_pv;
    return sv - pv;
#endif
}

static int32_t app_fb_controller_predictive_brake_blend(
    int32_t predicted_error,
    int32_t normal_pwm,
    int32_t boosted_pwm)
{
#if APP_FB_PREDICTIVE_BRAKE_ENABLE
    int64_t span;
    int64_t x_q15;
    int64_t smooth_q15;
    int64_t scale;
    int64_t target;

    normal_pwm = APP_FB_LIMIT(normal_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
    boosted_pwm = APP_FB_LIMIT(boosted_pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);

    if(predicted_error >= APP_FB_PREDICTIVE_BRAKE_ENTER_ERROR)
        return boosted_pwm;
    if(predicted_error <= APP_FB_PREDICTIVE_BRAKE_FULL_ERROR)
        return normal_pwm;

    span = (int64_t)APP_FB_PREDICTIVE_BRAKE_ENTER_ERROR -
           (int64_t)APP_FB_PREDICTIVE_BRAKE_FULL_ERROR;
    scale = (int64_t)APP_FB_FAST_HEAT_BLEND_SCALE;
    if(span <= 0 || scale <= 0) return normal_pwm;

    x_q15 = (((int64_t)predicted_error - (int64_t)APP_FB_PREDICTIVE_BRAKE_FULL_ERROR) * scale) / span;
    if(x_q15 < 0) x_q15 = 0;
    if(x_q15 > scale) x_q15 = scale;

    smooth_q15 = (x_q15 * x_q15 * ((3 * scale) - (2 * x_q15))) / (scale * scale);
    target = (int64_t)normal_pwm +
             (smooth_q15 * ((int64_t)boosted_pwm - (int64_t)normal_pwm)) / scale;
    return app_fb_controller_limit_i64(target, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
#else
    (void)predicted_error;
    (void)normal_pwm;
    return boosted_pwm;
#endif
}

static APP_FB_BOOL app_fb_controller_sample_time_valid(uint32_t sample_time_ms)
{
    if(sample_time_ms < APP_FB_SAMPLE_TIME_MIN_MS) return APP_FB_FALSE;
    if(sample_time_ms > APP_FB_SAMPLE_TIME_MAX_MS) return APP_FB_FALSE;
    return APP_FB_TRUE;
}

static APP_FB_ERROR app_fb_controller_apply_sample_time(APP_FB_TEMPERATURE_CONTROLLER_T *fb, uint32_t sample_time_ms)
{
    uint64_t sample_time_us;
    if(fb == 0) return APP_FB_ERROR_NULL_POINTER;
    if(app_fb_controller_sample_time_valid(sample_time_ms) == APP_FB_FALSE) return APP_FB_ERROR_PARAMETER;

    sample_time_us = (uint64_t)sample_time_ms * (uint64_t)APP_FB_SAMPLE_TIME_US_PER_MS;
    if(sample_time_us > UINT32_MAX) return APP_FB_ERROR_PARAMETER;

    fb->timing.sample_time_ms = sample_time_ms;
    fb->sample_time_us = (uint32_t)sample_time_us;
    return APP_FB_OK;
}

MY_API void app_fb_temperature_controller_init(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter)
{
    APP_FB_TIMING_PARAMETER_T timing;
    timing.sample_time_ms = APP_FB_SAMPLE_TIME_DEFAULT_MS;
    (void)app_fb_temperature_controller_init_ex_timed(fb, ff_table, ff_size, pid_parameter, 0, &timing);
}

MY_API void app_fb_temperature_controller_init_ex(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter)
{
    APP_FB_TIMING_PARAMETER_T timing;
    timing.sample_time_ms = APP_FB_SAMPLE_TIME_DEFAULT_MS;
    (void)app_fb_temperature_controller_init_ex_timed(fb, ff_table, ff_size, pid_parameter, adaptive_parameter, &timing);
}

MY_API APP_FB_ERROR app_fb_temperature_controller_init_timed(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_TIMING_PARAMETER_T *timing_parameter)
{
    return app_fb_temperature_controller_init_ex_timed(fb, ff_table, ff_size, pid_parameter, 0, timing_parameter);
}

MY_API APP_FB_ERROR app_fb_temperature_controller_init_ex_timed(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter,
    const APP_FB_TIMING_PARAMETER_T *timing_parameter)
{
    APP_FB_ERROR timing_status;
    APP_FB_ERROR pid_status;
    APP_FB_ERROR d_filter_status;
    APP_FB_ERROR rate_limit_status;
    APP_FB_ERROR learning_status;

    if(fb == 0 || timing_parameter == 0 || pid_parameter == 0) return APP_FB_ERROR_NULL_POINTER;

    timing_status = app_fb_controller_apply_sample_time(fb, timing_parameter->sample_time_ms);
    if(timing_status != APP_FB_OK) return timing_status;

    pid_status = app_fb_pid_init_timed(&fb->pid, pid_parameter, fb->timing.sample_time_ms);
    if(pid_status != APP_FB_OK) return pid_status;

    app_fb_feedforward_init(&fb->ff, ff_table, ff_size);

    d_filter_status = app_fb_d_filter_init_timed(&fb->d_filter, fb->timing.sample_time_ms, APP_FB_D_FILTER_TIME_CONSTANT_MS);
    if(d_filter_status != APP_FB_OK) return d_filter_status;

    app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);

    rate_limit_status = app_fb_rate_limit_init_timed(&fb->rate_limit, fb->timing.sample_time_ms,
                                                      APP_FB_PWM_RISE_RATE_PER_SEC, APP_FB_PWM_FALL_RATE_PER_SEC);
    if(rate_limit_status != APP_FB_OK) return rate_limit_status;

    learning_status = app_fb_ff_learning_init_timed(&fb->learning, adaptive_parameter, fb->timing.sample_time_ms);
    if(learning_status != APP_FB_OK) return learning_status;

    fb->previous_pwm = 0;
    fb->previous_sv = 0;
    fb->manual_active = APP_FB_FALSE;
    fb->sv_initialized = APP_FB_FALSE;
    fb->integral_disturbance_armed = APP_FB_FALSE;
    fb->fast_heat_active = APP_FB_FALSE;
    fb->predictive_brake_active = APP_FB_FALSE;
    fb->state = APP_FB_STATE_IDLE;
    return APP_FB_OK;
}

MY_API uint32_t app_fb_temperature_controller_get_sample_time_ms(const APP_FB_TEMPERATURE_CONTROLLER_T *fb)
{
    if(fb == 0) return 0U;
    return fb->timing.sample_time_ms;
}

MY_API uint32_t app_fb_temperature_controller_get_sample_time_us(const APP_FB_TEMPERATURE_CONTROLLER_T *fb)
{
    if(fb == 0) return 0U;
    return fb->sample_time_us;
}

MY_API void app_fb_temperature_controller_set_adaptive_parameter(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter)
{
    if(fb == 0) return;
    (void)app_fb_ff_learning_reconfigure_timed(&fb->learning, adaptive_parameter, fb->timing.sample_time_ms);
}

MY_API void app_fb_temperature_controller_reset(APP_FB_TEMPERATURE_CONTROLLER_T *fb)
{
    if(fb == 0) return;
    app_fb_pid_reset(&fb->pid);
    if(fb->ff.table != 0 && fb->ff.size > 0) app_fb_feedforward_init(&fb->ff, fb->ff.table, fb->ff.size);
    else fb->ff.output = 0;
    app_fb_d_filter_reset(&fb->d_filter);
    app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
    app_fb_rate_limit_reset(&fb->rate_limit, 0);
    app_fb_ff_learning_reset(&fb->learning);

    fb->previous_pwm = 0;
    fb->previous_sv = 0;
    fb->manual_active = APP_FB_FALSE;
    fb->sv_initialized = APP_FB_FALSE;
    fb->integral_disturbance_armed = APP_FB_FALSE;
    fb->fast_heat_active = APP_FB_FALSE;
    fb->predictive_brake_active = APP_FB_FALSE;
    fb->state = APP_FB_STATE_IDLE;
}

MY_API void app_fb_temperature_controller_run(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    int32_t ff_base_pwm, ff_offset, pid_raw, total_raw, pid_limited_pwm;
    int32_t target_pwm, actual_pwm, error, d_filtered, predicted_error;
    int32_t desired_pid_output, boosted_pwm;
    int64_t desired_pid_output64, total_raw64, sv_delta;
    APP_FB_BOOL bumpless_transition, learning_allowed, fast_heat_active;
    APP_FB_BOOL predictive_brake_active;

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
        fb->fast_heat_active = APP_FB_FALSE;
        fb->predictive_brake_active = APP_FB_FALSE;
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
        fb->fast_heat_active = APP_FB_FALSE;
        fb->predictive_brake_active = APP_FB_FALSE;
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
            fb->fast_heat_active = APP_FB_FALSE;
            fb->predictive_brake_active = APP_FB_FALSE;
            app_fb_integral_separation_init(&fb->i_sep, APP_FB_I_ENABLE_ERROR);
            app_fb_pid_reset(&fb->pid);
        }
    }

    error = input->sv - input->pv;
    output->error = error;
    fast_heat_active = app_fb_controller_fast_heat_state_update(fb, error);

    if(app_fb_controller_abs_i64((int64_t)error) <= (int64_t)APP_FB_I_ENABLE_ERROR)
        fb->integral_disturbance_armed = APP_FB_TRUE;

    ff_base_pwm = app_fb_feedforward_run(&fb->ff, input->sv);
    ff_offset = app_fb_ff_learning_get_offset(&fb->learning);
    output->ff_pwm = ff_base_pwm;
    output->ff_offset = ff_offset;

    d_filtered = app_fb_d_filter_run(&fb->d_filter, input->pv);
    predicted_error = app_fb_controller_predict_error(fb, input->sv, input->pv, d_filtered);
    predictive_brake_active = APP_FB_FALSE;
#if APP_FB_PREDICTIVE_BRAKE_ENABLE
    if(fast_heat_active == APP_FB_TRUE &&
       d_filtered >= APP_FB_PREDICTIVE_BRAKE_MIN_RISE_DELTA &&
       predicted_error <= APP_FB_PREDICTIVE_BRAKE_ENTER_ERROR)
    {
        predictive_brake_active = APP_FB_TRUE;
    }
#endif
    fb->predictive_brake_active = predictive_brake_active;
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
    else if(error >= APP_FB_INTEGRAL_FREEZE_ERROR)
    {
        fb->pid.integral_enable = APP_FB_FALSE;
    }
    else if(error > APP_FB_I_ENABLE_ERROR)
    {
        fb->pid.integral_enable = APP_FB_TRUE;
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
    pid_limited_pwm = APP_FB_LIMIT(total_raw, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
    target_pwm = pid_limited_pwm;

    if(fast_heat_active == APP_FB_TRUE)
    {
        boosted_pwm = app_fb_controller_fast_heat_blend(error, pid_limited_pwm);
        boosted_pwm = app_fb_controller_fast_heat_tail_boost(error, boosted_pwm);

        /* Predictive Brake is the final authority and may suppress both main and tail boost. */
        if(predictive_brake_active == APP_FB_TRUE)
            target_pwm = app_fb_controller_predictive_brake_blend(predicted_error, pid_limited_pwm, boosted_pwm);
        else
            target_pwm = boosted_pwm;
    }

    actual_pwm = app_fb_rate_limit_run(&fb->rate_limit, target_pwm);
    output->pwm = actual_pwm;
    app_fb_pid_anti_windup(&fb->pid, total_raw, pid_limited_pwm);

    learning_allowed = APP_FB_FALSE;
    if(fast_heat_active == APP_FB_FALSE && predictive_brake_active == APP_FB_FALSE &&
       total_raw >= APP_FB_PWM_MIN && total_raw <= APP_FB_PWM_MAX &&
       actual_pwm == pid_limited_pwm && bumpless_transition == APP_FB_FALSE)
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
