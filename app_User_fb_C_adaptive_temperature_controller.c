/***************************************************************
Description :
	This is a user USER_ADAPTIVE_TEMP_CONTROLLER program application.

Change notice:
Date-> 2026/05/13
[ADD] 1. The first version sets up.
[MODIFY] 1. Connect derivative filter output to PID.
[MODIFY] 2. Move previous-PV ownership from PID/controller into derivative filter.
[MODIFY] 3. Connect actual final PWM to PID back-calculation anti-windup.
[MODIFY] 4. Use configurable PID Kaw for discrete-time anti-windup.
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
    if(fb == 0)
        return;

    app_fb_feedforward_init(&fb->ff, ff_table, ff_size);

    if(pid_parameter != 0)
        app_fb_pid_init(&fb->pid, pid_parameter);
    else
        app_fb_pid_init(&fb->pid, &default_pid);

    app_fb_d_filter_init(&fb->d_filter, APP_FB_D_FILTER_ALPHA);

    app_fb_integral_separation_init(
        &fb->i_sep,
        APP_FB_I_ENABLE_ERROR
    );

    app_fb_rate_limit_init(
        &fb->rate_limit,
        APP_FB_PWM_RISE_LIMIT,
        APP_FB_PWM_FALL_LIMIT
    );

    app_fb_ff_learning_init(&fb->learning, 0);

    fb->previous_pwm = 0;
    fb->state = APP_FB_STATE_IDLE;
}

void app_fb_temperature_controller_reset
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb
)
{
    if(fb == 0)
        return;

    app_fb_pid_reset(&fb->pid);
    app_fb_d_filter_reset(&fb->d_filter);
    app_fb_ff_learning_reset(&fb->learning);
    app_fb_rate_limit_reset(&fb->rate_limit, 0);

    fb->previous_pwm = 0;
    fb->state = APP_FB_STATE_IDLE;
}

/*
====================================================
 Main Controller Execute 50Hz
====================================================
*/
void app_fb_temperature_controller_run
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output
)
{
    int32_t ff_pwm;
    int32_t pid_pwm;
    int32_t unsaturated_command;
    int32_t limited_command;
    int32_t limited_pwm;
    int32_t error;
    int32_t d_filtered;

    if(fb == 0 || input == 0 || output == 0)
        return;

    output->pwm = 0;
    output->ff_pwm = 0;
    output->pid_output = 0;
    output->ff_offset = 0;

    /* Enable Check */
    if(input->enable == APP_FB_FALSE)
    {
        fb->state = APP_FB_STATE_IDLE;
        return;
    }

    /* Manual Mode */
    if(input->mode == APP_FB_MODE_MANUAL)
    {
        output->pwm = APP_FB_LIMIT(
            input->manual_pwm,
            APP_FB_PWM_MIN,
            APP_FB_PWM_MAX
        );
        return;
    }

    /* Error */
    error = input->sv - input->pv;
    output->error = error;

    /* Feedforward */
    ff_pwm = app_fb_feedforward_run(&fb->ff, input->sv);

    ff_pwm += app_fb_ff_learning_get_offset(&fb->learning);

    ff_pwm = APP_FB_LIMIT(
        ff_pwm,
        APP_FB_PWM_MIN,
        APP_FB_PWM_MAX
    );

    output->ff_pwm = ff_pwm;

    /* Integral Separation */
    fb->pid.integral_enable = app_fb_integral_separation_run(
        &fb->i_sep,
        error
    );

    /*
     * Derivative Filter
     * The filter owns previous PV and calculates filtered dPV.
     * First execution after init/reset returns zero derivative.
     */
    d_filtered = app_fb_d_filter_run(
        &fb->d_filter,
        input->pv
    );

    /* PID */
    pid_pwm = app_fb_pid_run(
        &fb->pid,
        input->sv,
        input->pv,
        d_filtered
    );

    output->pid_output = pid_pwm;

    /*
     * Combine FF + PID.
     * Keep the unsaturated command for anti-windup feedback.
     */
    unsaturated_command = ff_pwm + pid_pwm;

    /* Physical PWM output limit. */
    limited_command = APP_FB_LIMIT(
        unsaturated_command,
        APP_FB_PWM_MIN,
        APP_FB_PWM_MAX
    );

    /* Output Rate Limit */
    limited_pwm = app_fb_rate_limit_run(
        &fb->rate_limit,
        limited_command
    );
    output->pwm = limited_pwm;

    /*
     * Back-calculation Anti-Windup.
     * Feed the final actual PWM back to the PID integral state.
     *
     * Using unsaturated_command here is intentional: the correction
     * includes both hard output saturation and the rate limiter.
     */
    app_fb_pid_anti_windup(
        &fb->pid,
        unsaturated_command,
        limited_pwm
    );

    /* Adaptive Learning */
    app_fb_ff_learning_run(
        &fb->learning,
        input->sv,
        input->pv,
        pid_pwm
    );

    output->ff_offset = app_fb_ff_learning_get_offset(
        &fb->learning
    );

    fb->state = APP_FB_STATE_RUN;
}

#ifdef __cplusplus
}
#endif
