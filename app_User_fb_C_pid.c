/***************************************************************
Description : PID FB

Step 6:
[MODIFY] 1. PID output is now a true raw controller output.
[MODIFY] 2. Actuator 0..1000 limit is handled by the temperature controller.
[MODIFY] 3. Anti-windup is based on hard PWM saturation only.

Step 9-2:
[MODIFY] 4. Keep standard back-calculation scaling for the discrete integral state.
[MODIFY] 5. Use int64_t for saturation error and correction arithmetic.
***************************************************************/

#include "app_User_fb_C_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_pid_limit(int32_t value, int32_t min, int32_t max)
{
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

void app_fb_pid_init(APP_FB_PID_T *fb, const APP_FB_PID_PARAMETER_T *param)
{
    if(fb == 0) return;

    if(param != 0)
        fb->param = *param;

    app_fb_pid_reset(fb);
    fb->enable = APP_FB_TRUE;
    fb->integral_enable = APP_FB_FALSE;
}

void app_fb_pid_reset(APP_FB_PID_T *fb)
{
    if(fb == 0) return;

    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.output = 0;
    fb->integral_enable = APP_FB_FALSE;
}

int32_t app_fb_pid_run(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv,
    int32_t d_filtered)
{
    int32_t error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int64_t raw_output;
    int64_t temp;

    if(fb == 0 || fb->enable == APP_FB_FALSE)
        return 0;

    error = sv - pv;

    /* P = Kp * error / 32768 */
    temp = (int64_t)fb->param.kp * error;
    p_term = (int32_t)(temp >> 15);

    /* Integral Separation is controlled by the caller. */
    if(fb->integral_enable == APP_FB_TRUE)
    {
        fb->state.integral += error;
        fb->state.integral = app_fb_pid_limit(
            fb->state.integral,
            -fb->param.integral_limit,
            fb->param.integral_limit);
    }

    /* I = Ki * accumulated error / 32768 */
    temp = (int64_t)fb->param.ki * fb->state.integral;
    i_term = (int32_t)(temp >> 15);

    /* D-on-measurement. d_filtered is supplied by D-filter FB. */
    temp = (int64_t)fb->param.kd * d_filtered;
    d_term = -(int32_t)(temp >> 15);

    /* IMPORTANT: no actuator/output clamp here. */
    raw_output = (int64_t)p_term + (int64_t)i_term + (int64_t)d_term;

    if(raw_output > INT32_MAX)
        raw_output = INT32_MAX;
    else if(raw_output < INT32_MIN)
        raw_output = INT32_MIN;

    fb->state.error_previous = error;
    fb->state.output = (int32_t)raw_output;

    return fb->state.output;
}

void app_fb_pid_anti_windup(
    APP_FB_PID_T *fb,
    int32_t unsaturated_output,
    int32_t actual_output)
{
    int64_t delta;
    int64_t correction;
    int64_t kaw;
    int64_t ki;
    int64_t integral;
    int64_t integral_limit;

    if(fb == 0 || fb->enable == APP_FB_FALSE)
        return;

    /* Integral Separation remains the authority for I enable. */
    if(fb->integral_enable == APP_FB_FALSE)
        return;

    kaw = fb->param.kaw;
    ki = fb->param.ki;
    integral_limit = (int64_t)fb->param.integral_limit;

    if(kaw == 0 || ki == 0 || integral_limit <= 0)
        return;

    /*
     * Hard saturation error only:
     *     delta = u_sat - u_raw
     *
     * Rate limiting is deliberately excluded from anti-windup.
     * int64_t is required because u_raw is a full int32_t raw output.
     */
    delta = (int64_t)actual_output - (int64_t)unsaturated_output;
    if(delta == 0)
        return;

    /*
     * The integral state is stored as accumulated error samples.
     * Since Iout = Ki * I / 32768 and Kaw is Q15:
     *
     *     Delta_I = Kaw / Ki * (u_sat - u_raw)
     *
     * This is the standard back-calculation form for this discrete PID.
     */
    correction = (kaw * delta) / ki;

    /* Preserve direction for sub-count corrections lost to integer division. */
    if(correction == 0)
        correction = (delta > 0) ? 1 : -1;

    integral = (int64_t)fb->state.integral + correction;

    if(integral > integral_limit)
        integral = integral_limit;
    else if(integral < -integral_limit)
        integral = -integral_limit;

    fb->state.integral = (int32_t)integral;
}

void app_fb_pid_integral_add(APP_FB_PID_T *fb, int32_t value)
{
    int64_t integral;
    int64_t limit;

    if(fb == 0) return;

    limit = (int64_t)fb->param.integral_limit;
    integral = (int64_t)fb->state.integral + (int64_t)value;

    if(integral > limit)
        integral = limit;
    else if(integral < -limit)
        integral = -limit;

    fb->state.integral = (int32_t)integral;
}

#ifdef __cplusplus
}
#endif
