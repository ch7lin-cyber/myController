#include <stdint.h>
#include "app_User_fb_C_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_fb_pid_init
(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param
)
{
    if(fb == 0)
        return;

    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.output = 0;
    fb->state.aw_remainder = 0;
    fb->enable = APP_FB_TRUE;
    fb->integral_enable = APP_FB_TRUE;

    if(param != 0)
        fb->param = *param;
}

void app_fb_pid_reset
(
    APP_FB_PID_T *fb
)
{
    if(fb == 0)
        return;

    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.output = 0;
    fb->state.aw_remainder = 0;
}

int32_t app_fb_pid_run
(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv,
    int32_t d_filtered
)
{
    int32_t error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int64_t integral_candidate;
    int64_t raw_output;

    if(fb == 0)
        return 0;

    if(fb->enable == APP_FB_FALSE)
        return 0;

    error = sv - pv;

    /* Integral accumulation is performed in int64_t before limiting. */
    if(fb->integral_enable == APP_FB_TRUE)
    {
        integral_candidate =
            (int64_t)fb->state.integral +
            (int64_t)error;

        if(integral_candidate > (int64_t)fb->param.integral_limit)
            integral_candidate = (int64_t)fb->param.integral_limit;
        else if(integral_candidate < -(int64_t)fb->param.integral_limit)
            integral_candidate = -(int64_t)fb->param.integral_limit;

        fb->state.integral = (int32_t)integral_candidate;
    }

    p_term = (int32_t)(((int64_t)fb->param.kp * error) >> 15);
    i_term = (int32_t)(((int64_t)fb->param.ki * fb->state.integral) >> 15);
    d_term = (int32_t)(((int64_t)fb->param.kd * d_filtered) >> 15);

    raw_output = (int64_t)p_term +
                 (int64_t)i_term +
                 (int64_t)d_term;

    if(raw_output > INT32_MAX)
        raw_output = INT32_MAX;
    else if(raw_output < INT32_MIN)
        raw_output = INT32_MIN;

    fb->state.output = (int32_t)raw_output;
    fb->state.error_previous = error;

    return fb->state.output;
}

void app_fb_pid_anti_windup
(
    APP_FB_PID_T *fb,
    int32_t unsaturated_output,
    int32_t actual_output
)
{
    int64_t delta;
    int64_t numerator;
    int64_t correction;
    int64_t integral_candidate;
    int64_t limited_correction;
    int32_t kaw;
    int32_t ki;
    int32_t limit;

    if(fb == 0)
        return;

    if(fb->enable == APP_FB_FALSE ||
       fb->integral_enable == APP_FB_FALSE)
    {
        fb->state.aw_remainder = 0;
        return;
    }

    kaw = fb->param.kaw;
    ki = fb->param.ki;
    limit = fb->param.integral_limit;

    if(kaw <= 0 || ki <= 0 || limit <= 0)
    {
        fb->state.aw_remainder = 0;
        return;
    }

    delta =
        (int64_t)actual_output -
        (int64_t)unsaturated_output;

    if(delta == 0)
    {
        fb->state.aw_remainder = 0;
        return;
    }

    /*
     * Keep aw_remainder strictly in numerator units (Kaw * PWM counts).
     *
     *     numerator   = Kaw * delta + remainder
     *     correction  = numerator / Ki
     *     remainder   = numerator % Ki
     *
     * The correction limit is applied only after conversion to integral
     * state units. The clipped portion is intentionally NOT added back to
     * aw_remainder because it is not a fractional division remainder.
     * The saturation error is evaluated again on the next controller cycle.
     */
    numerator =
        ((int64_t)kaw * delta) +
        fb->state.aw_remainder;

    correction = numerator / (int64_t)ki;
    fb->state.aw_remainder = numerator % (int64_t)ki;

    limited_correction = correction;
    if(limited_correction > (int64_t)APP_FB_PID_AW_MAX_CORRECTION)
        limited_correction = (int64_t)APP_FB_PID_AW_MAX_CORRECTION;
    else if(limited_correction < -(int64_t)APP_FB_PID_AW_MAX_CORRECTION)
        limited_correction = -(int64_t)APP_FB_PID_AW_MAX_CORRECTION;

    integral_candidate =
        (int64_t)fb->state.integral +
        limited_correction;

    if(integral_candidate > (int64_t)limit)
        integral_candidate = (int64_t)limit;
    else if(integral_candidate < -(int64_t)limit)
        integral_candidate = -(int64_t)limit;

    fb->state.integral = (int32_t)integral_candidate;

    /* Integral limit is authoritative; discard stale fractional state. */
    if(fb->state.integral == limit ||
       fb->state.integral == -limit)
    {
        fb->state.aw_remainder = 0;
    }
}

void app_fb_pid_integral_add
(
    APP_FB_PID_T *fb,
    int32_t value
)
{
    int64_t integral_candidate;

    if(fb == 0)
        return;

    integral_candidate =
        (int64_t)fb->state.integral +
        (int64_t)value;

    if(integral_candidate > (int64_t)fb->param.integral_limit)
        integral_candidate = (int64_t)fb->param.integral_limit;
    else if(integral_candidate < -(int64_t)fb->param.integral_limit)
        integral_candidate = -(int64_t)fb->param.integral_limit;

    fb->state.integral = (int32_t)integral_candidate;
    fb->state.aw_remainder = 0;
}

#ifdef __cplusplus
}
#endif
