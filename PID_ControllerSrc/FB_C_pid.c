#include <stdint.h>
#include <limits.h>
#include "FB_C_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_pid_scale_ratio_i32(
    int32_t value,
    uint32_t numerator,
    uint32_t denominator)
{
    int64_t product;
    int64_t scaled;

    if(denominator == 0U) return 0;

    product = (int64_t)value * (int64_t)numerator;

    /* Round to nearest while preserving sign. */
    if(product >= 0)
        scaled = (product + ((int64_t)denominator / 2)) / (int64_t)denominator;
    else
        scaled = (product - ((int64_t)denominator / 2)) / (int64_t)denominator;

    if(scaled > INT32_MAX) return INT32_MAX;
    if(scaled < INT32_MIN) return INT32_MIN;
    return (int32_t)scaled;
}

static int32_t app_fb_pid_keep_nonzero_sign(int32_t reference, int32_t scaled)
{
    if(reference > 0 && scaled == 0) return 1;
    if(reference < 0 && scaled == 0) return -1;
    return scaled;
}

static APP_FB_ERROR app_fb_pid_build_runtime_parameter(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param,
    uint32_t sample_time_ms)
{
    int32_t runtime_integral_limit;

    if(fb == 0 || param == 0) return APP_FB_ERROR_NULL_POINTER;
    if(sample_time_ms < APP_FB_SAMPLE_TIME_MIN_MS ||
       sample_time_ms > APP_FB_SAMPLE_TIME_MAX_MS)
    {
        return APP_FB_ERROR_PARAMETER;
    }

    fb->param = *param;
    fb->runtime_param = *param;
    fb->sample_time_ms = sample_time_ms;

    /* Kp is independent of sample period. */
    fb->runtime_param.kp = param->kp;

    /*
     * Existing public gains are reference/discrete gains at 20 ms.
     * Normalize them once at initialization for the actual fixed scheduler Ts.
     */
    fb->runtime_param.ki = app_fb_pid_keep_nonzero_sign(
        param->ki,
        app_fb_pid_scale_ratio_i32(
            param->ki,
            sample_time_ms,
            APP_FB_PID_REFERENCE_SAMPLE_TIME_MS));

    fb->runtime_param.kd = app_fb_pid_keep_nonzero_sign(
        param->kd,
        app_fb_pid_scale_ratio_i32(
            param->kd,
            APP_FB_PID_REFERENCE_SAMPLE_TIME_MS,
            sample_time_ms));

    /*
     * Integral state is stored as accumulated error samples. Scale its state
     * limit inversely with Ts so the maximum I-term authority remains close to
     * the 20 ms reference controller:
     *     Ki_runtime * I_limit_runtime ~= Ki_ref * I_limit_ref
     */
    runtime_integral_limit = app_fb_pid_scale_ratio_i32(
        param->integral_limit,
        APP_FB_PID_REFERENCE_SAMPLE_TIME_MS,
        sample_time_ms);

    if(param->integral_limit > 0 && runtime_integral_limit < 1)
        runtime_integral_limit = 1;
    else if(runtime_integral_limit < 0)
        runtime_integral_limit = 0;

    fb->runtime_param.integral_limit = runtime_integral_limit;

    /* Output clamp is a physical PWM limit and is independent of Ts. */
    fb->runtime_param.output_limit = param->output_limit;

    /*
     * Kaw is a discrete-time anti-windup gain. Scale it with Ts just like Ki;
     * the Kaw/Ki ratio therefore remains consistent with the reference loop.
     */
    fb->runtime_param.kaw = app_fb_pid_keep_nonzero_sign(
        param->kaw,
        app_fb_pid_scale_ratio_i32(
            param->kaw,
            sample_time_ms,
            APP_FB_PID_REFERENCE_SAMPLE_TIME_MS));

    fb->aw_max_correction_runtime = APP_FB_PID_AW_MAX_CORRECTION;

    return APP_FB_OK;
}

void app_fb_pid_init(APP_FB_PID_T *fb, const APP_FB_PID_PARAMETER_T *param)
{
    (void)app_fb_pid_init_timed(
        fb,
        param,
        APP_FB_PID_REFERENCE_SAMPLE_TIME_MS);
}

APP_FB_ERROR app_fb_pid_init_timed(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param,
    uint32_t sample_time_ms)
{
    APP_FB_ERROR status;

    if(fb == 0 || param == 0) return APP_FB_ERROR_NULL_POINTER;

    status = app_fb_pid_build_runtime_parameter(fb, param, sample_time_ms);
    if(status != APP_FB_OK) return status;

    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.output = 0;
    fb->state.aw_remainder = 0;
    fb->enable = APP_FB_TRUE;
    fb->integral_enable = APP_FB_TRUE;

    return APP_FB_OK;
}

void app_fb_pid_reset(APP_FB_PID_T *fb)
{
    if(fb == 0) return;
    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.output = 0;
    fb->state.aw_remainder = 0;
}

int32_t app_fb_pid_run(APP_FB_PID_T *fb, APP_FB_TEMP sv, APP_FB_TEMP pv, int32_t d_filtered)
{
    int32_t error, p_term, i_term, d_term;
    int64_t integral_candidate, raw_output;
    const APP_FB_PID_PARAMETER_T *runtime;

    if(fb == 0 || fb->enable == APP_FB_FALSE) return 0;

    runtime = &fb->runtime_param;
    error = sv - pv;

    if(fb->integral_enable == APP_FB_TRUE)
    {
        integral_candidate = (int64_t)fb->state.integral + (int64_t)error;
        if(integral_candidate > (int64_t)runtime->integral_limit)
            integral_candidate = (int64_t)runtime->integral_limit;
        else if(integral_candidate < -(int64_t)runtime->integral_limit)
            integral_candidate = -(int64_t)runtime->integral_limit;
        fb->state.integral = (int32_t)integral_candidate;
    }

    p_term = (int32_t)(((int64_t)runtime->kp * error) >> 15);
    i_term = (int32_t)(((int64_t)runtime->ki * fb->state.integral) >> 15);
    d_term = (int32_t)(((int64_t)runtime->kd * d_filtered) >> 15);

    raw_output = (int64_t)p_term + (int64_t)i_term + (int64_t)d_term;
    if(raw_output > INT32_MAX) raw_output = INT32_MAX;
    else if(raw_output < INT32_MIN) raw_output = INT32_MIN;

    fb->state.output = (int32_t)raw_output;
    fb->state.error_previous = error;
    return fb->state.output;
}

void app_fb_pid_bumpless_preload(APP_FB_PID_T *fb, APP_FB_TEMP sv, APP_FB_TEMP pv, int32_t d_filtered, int32_t desired_pid_output)
{
    int32_t error;
    int64_t p_term, d_term, desired_i_output;
    int64_t integral_after_run, integral_before_run;
    const APP_FB_PID_PARAMETER_T *runtime;

    if(fb == 0) return;
    runtime = &fb->runtime_param;
    if(runtime->ki <= 0) return;

    error = sv - pv;
    p_term = ((int64_t)runtime->kp * (int64_t)error) >> 15;
    d_term = ((int64_t)runtime->kd * (int64_t)d_filtered) >> 15;
    desired_i_output = (int64_t)desired_pid_output - p_term - d_term;
    integral_after_run = (desired_i_output * (int64_t)APP_FB_Q15_ONE) /
                         (int64_t)runtime->ki;
    integral_before_run = integral_after_run - (int64_t)error;

    if(integral_before_run > (int64_t)runtime->integral_limit)
        integral_before_run = (int64_t)runtime->integral_limit;
    else if(integral_before_run < -(int64_t)runtime->integral_limit)
        integral_before_run = -(int64_t)runtime->integral_limit;

    fb->state.integral = (int32_t)integral_before_run;
    fb->state.aw_remainder = 0;
    fb->state.error_previous = error;
}

void app_fb_pid_anti_windup(APP_FB_PID_T *fb, int32_t unsaturated_output, int32_t actual_output)
{
    int64_t delta, numerator, correction, integral_candidate, limited_correction;
    int32_t kaw, ki, limit;
    const APP_FB_PID_PARAMETER_T *runtime;

    if(fb == 0) return;
    if(fb->enable == APP_FB_FALSE || fb->integral_enable == APP_FB_FALSE)
    {
        fb->state.aw_remainder = 0;
        return;
    }

    runtime = &fb->runtime_param;
    kaw = runtime->kaw;
    ki = runtime->ki;
    limit = runtime->integral_limit;

    if(kaw <= 0 || ki <= 0 || limit <= 0)
    {
        fb->state.aw_remainder = 0;
        return;
    }

    delta = (int64_t)actual_output - (int64_t)unsaturated_output;
    if(delta == 0)
    {
        fb->state.aw_remainder = 0;
        return;
    }

    numerator = ((int64_t)kaw * delta) + fb->state.aw_remainder;
    correction = numerator / (int64_t)ki;
    fb->state.aw_remainder = numerator % (int64_t)ki;

    limited_correction = correction;
    if(limited_correction > (int64_t)fb->aw_max_correction_runtime)
        limited_correction = (int64_t)fb->aw_max_correction_runtime;
    else if(limited_correction < -(int64_t)fb->aw_max_correction_runtime)
        limited_correction = -(int64_t)fb->aw_max_correction_runtime;

    integral_candidate = (int64_t)fb->state.integral + limited_correction;
    if(integral_candidate > (int64_t)limit)
        integral_candidate = (int64_t)limit;
    else if(integral_candidate < -(int64_t)limit)
        integral_candidate = -(int64_t)limit;

    fb->state.integral = (int32_t)integral_candidate;
    if(fb->state.integral == limit || fb->state.integral == -limit)
        fb->state.aw_remainder = 0;
}

void app_fb_pid_integral_add(APP_FB_PID_T *fb, int32_t value)
{
    int64_t integral_candidate;
    int32_t limit;

    if(fb == 0) return;

    limit = fb->runtime_param.integral_limit;
    integral_candidate = (int64_t)fb->state.integral + (int64_t)value;

    if(integral_candidate > (int64_t)limit)
        integral_candidate = (int64_t)limit;
    else if(integral_candidate < -(int64_t)limit)
        integral_candidate = -(int64_t)limit;

    fb->state.integral = (int32_t)integral_candidate;
    fb->state.aw_remainder = 0;
}

#ifdef __cplusplus
}
#endif
