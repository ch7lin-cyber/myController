/***************************************************************
Description :
	This is a user User fb C program application.

Change notice:
Date-> 2026/05/13
[ADD] 1. The first version sets up.
[MODIFY] 1. Connect filtered derivative input to PID.
[MODIFY] 2. Remove previous-PV ownership from PID; derivative filter owns it.
[MODIFY] 3. Add back-calculation anti-windup.
***************************************************************/

#include "app_User_fb_C_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_pid_limit
(
    int32_t value,
    int32_t min,
    int32_t max
)
{
    if(value > max)
        return max;
    if(value < min)
        return min;
    return value;
}

void app_fb_pid_init
(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param
)
{
    if(fb == 0)
        return;

    if(param != 0)
        fb->param = *param;

    app_fb_pid_reset(fb);
    fb->enable = APP_FB_TRUE;
    fb->integral_enable = APP_FB_FALSE;
}

void app_fb_pid_reset( APP_FB_PID_T *fb )
{
    if(fb == 0)
        return;

    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.output = 0;
    fb->integral_enable = APP_FB_FALSE;
}

/*
====================================================
 PID Execute
 PID = P + I + D

 Integral Separation is controlled by the caller through
 fb->integral_enable.

 Derivative is D-on-measurement and is supplied by the
 derivative-filter FB. The PID FB does not own previous PV.
====================================================
*/
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
    int32_t output;
    int64_t temp;

    if(fb == 0)
        return 0;

    if(fb->enable == APP_FB_FALSE)
        return 0;

    error = sv - pv;

    /* P = Kp * Error */
    temp = (int64_t)fb->param.kp * error;
    p_term = (int32_t)(temp >> 15);

    /* Integral Separation is decided by the controller-level FB. */
    if(fb->integral_enable == APP_FB_TRUE)
    {
        fb->state.integral += error;
        fb->state.integral = app_fb_pid_limit
        (
            fb->state.integral,
            -fb->param.integral_limit,
            fb->param.integral_limit
        );
    }

    temp = (int64_t)fb->param.ki * fb->state.integral;
    i_term = (int32_t)(temp >> 15);

    /* D = -Kd * filtered dPV */
    temp = (int64_t)fb->param.kd * d_filtered;
    d_term = -(int32_t)(temp >> 15);

    output = p_term + i_term + d_term;

    /* PID correction output limit. */
    output = app_fb_pid_limit
    (
        output,
        -fb->param.output_limit,
        fb->param.output_limit
    );

    fb->state.error_previous = error;
    fb->state.output = output;

    return output;
}

/*
====================================================
 Back Calculation Anti-Windup

 The controller calls this after the final actual PWM is
 available. The difference between actual and unsaturated
 total command is fed back to the PID integral accumulator.

     integral += Kaw / Ki * (actual - unsaturated)

 Kaw is Q15 and comes from APP_FB_KAW.

 Integral Separation remains the authority for deciding when
 the integral is active. Therefore the correction is applied
 only while integral is enabled.
====================================================
*/
void app_fb_pid_anti_windup
(
    APP_FB_PID_T *fb,
    int32_t unsaturated_output,
    int32_t actual_output
)
{
    int32_t delta;
    int64_t correction;

    if(fb == 0)
        return;

    if(fb->enable == APP_FB_FALSE)
        return;

    if(fb->integral_enable == APP_FB_FALSE)
        return;

    if(fb->param.ki == 0)
        return;

    /* No saturation/rate-limit error: no correction required. */
    delta = actual_output - unsaturated_output;
    if(delta == 0)
        return;

    /*
     * PID integral output is:
     *     Iout = Ki * integral / 32768
     *
     * Desired Iout correction:
     *     Kaw * delta / 32768
     *
     * Therefore the integral-state correction is:
     *     Kaw * delta / Ki
     */
    correction = (int64_t)APP_FB_KAW * delta;
    correction /= fb->param.ki;

    /*
     * Avoid a small correction disappearing forever due to
     * integer truncation. Preserve at least one accumulator
     * count when the requested correction is non-zero.
     */
    if(correction == 0)
        correction = (delta > 0) ? 1 : -1;

    fb->state.integral += (int32_t)correction;

    fb->state.integral = app_fb_pid_limit
    (
        fb->state.integral,
        -fb->param.integral_limit,
        fb->param.integral_limit
    );
}

void app_fb_pid_integral_add
(
    APP_FB_PID_T *fb,
    int32_t value
)
{
    if(fb == 0)
        return;

    fb->state.integral += value;

    fb->state.integral = app_fb_pid_limit
    (
        fb->state.integral,
        -fb->param.integral_limit,
        fb->param.integral_limit
    );
}

#ifdef __cplusplus
}
#endif
