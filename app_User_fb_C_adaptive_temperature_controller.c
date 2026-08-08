#include "app_User_fb_C_adaptive_temperature_controller.h"
#include "app_User_fb_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_controller_limit_i64(int64_t value, int32_t min_value, int32_t max_value)
{
    if(value > (int64_t)max_value) return max_value;
    if(value < (int64_t)min_value) return min_value;
    return (int32_t)value;
}

/* Main Controller Execute: 50Hz */
void app_fb_temperature_controller_run(
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
    int64_t total_raw64;

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

    /* 4. Base Feedforward + adaptive offset */
    ff_base_pwm = app_fb_feedforward_run(&fb->ff, input->sv);
    ff_offset = app_fb_ff_learning_get_offset(&fb->learning);

    output->ff_pwm = ff_base_pwm;
    output->ff_offset = ff_offset;

    /* 5. Integral Separation */
    fb->pid.integral_enable = app_fb_integral_separation_run(&fb->i_sep, error);

    /* 6. Derivative Filter */
    d_filtered = app_fb_d_filter_run(&fb->d_filter, input->pv);

    /* 7. PID Raw Output - no actuator clamp in PID */
    pid_raw = app_fb_pid_run(&fb->pid, input->sv, input->pv, d_filtered);
    output->pid_output = pid_raw;

    /*
     * 8. FF + learning offset + PID.
     * Calculate in int64_t first so signed int32 overflow cannot occur
     * before the actuator saturation stage.
     */
    total_raw64 = (int64_t)ff_base_pwm +
                  (int64_t)ff_offset +
                  (int64_t)pid_raw;

    total_raw = app_fb_controller_limit_i64(
        total_raw64,
        INT32_MIN,
        INT32_MAX);

    /* 9. Hard PWM saturation: 0..1000 */
    pwm_limited = APP_FB_LIMIT(
        total_raw,
        APP_FB_PWM_MIN,
        APP_FB_PWM_MAX);

    /* 10. Rate Limit: slew constraint, not anti-windup saturation */
    actual_pwm = app_fb_rate_limit_run(&fb->rate_limit, pwm_limited);
    output->pwm = actual_pwm;

    /*
     * 11. Anti-Windup uses hard saturation only.
     * Do NOT use actual_pwm because it has already passed the
     * independent output rate limiter.
     */
    app_fb_pid_anti_windup(&fb->pid, total_raw, pwm_limited);

    /* 12. Adaptive Learning is gated inside the learning FB/controller. */
    if(total_raw >= APP_FB_PWM_MIN && total_raw <= APP_FB_PWM_MAX)
    {
        app_fb_ff_learning_run(&fb->learning, input->sv, input->pv, pid_raw);
    }

    output->ff_offset = app_fb_ff_learning_get_offset(&fb->learning);
    fb->state = APP_FB_STATE_RUN;
}

#ifdef __cplusplus
}
#endif
