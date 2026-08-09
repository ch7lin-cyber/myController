#include "FB_C_autotune_controller_binding.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

static void app_fb_autotune_binding_clear_output(
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    if(output == NULL) return;
    output->pwm = 0;
    output->ff_pwm = 0;
    output->pid_output = 0;
    output->ff_offset = 0;
    output->error = 0;
}

static void app_fb_autotune_binding_restore_learning(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller)
{
    if(fb == NULL || controller == NULL) return;
    app_fb_temperature_controller_set_learning_enabled(
        controller,
        fb->original_learning_enabled);
}

static void app_fb_autotune_binding_apply_candidate(
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    const APP_FB_PID_PARAMETER_T *pid,
    APP_FB_PWM handoff_pwm)
{
    if(controller == NULL || input == NULL || pid == NULL) return;

    /* Preserve the real actuator level across Relay -> PID and retry handoffs. */
    controller->previous_pwm = APP_FB_LIMIT(
        handoff_pwm,
        APP_FB_PWM_MIN,
        APP_FB_PWM_MAX);

    /* AutoTune is performed at one fixed SV. Synchronize the controller's SV
       tracker before loading the candidate so the first regression cycle does
       not interpret the tune SV as a fresh setpoint change and reset the PID. */
    controller->previous_sv = input->sv;
    controller->sv_initialized = APP_FB_TRUE;
    controller->manual_active = APP_FB_FALSE;
    controller->integral_disturbance_armed = APP_FB_FALSE;

    app_fb_temperature_controller_set_pid_parameter(controller, pid);
}

void app_fb_autotune_controller_binding_init(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    const APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T *cfg)
{
    if(fb == NULL) return;

    fb->initialized = APP_FB_FALSE;
    fb->final_pid_valid = APP_FB_FALSE;
    fb->original_learning_enabled = APP_FB_TRUE;
    fb->status = APP_FB_AUTOTUNE_BINDING_ERROR;

    fb->original_pid.kp = 0;
    fb->original_pid.ki = 0;
    fb->original_pid.kd = 0;
    fb->original_pid.integral_limit = 0;
    fb->original_pid.output_limit = 0;
    fb->original_pid.kaw = 0;
    fb->final_pid = fb->original_pid;

    if(cfg == NULL) return;

    app_fb_autotune_supervisor_init(&fb->supervisor, cfg);
    if(fb->supervisor.state == APP_FB_AUTOTUNE_SUPERVISOR_ERROR)
        return;

    fb->status = APP_FB_AUTOTUNE_BINDING_IDLE;
    fb->initialized = APP_FB_TRUE;
}

APP_FB_BOOL app_fb_autotune_controller_binding_start(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    APP_FB_TEMP initial_pv)
{
    if(fb == NULL || controller == NULL) return APP_FB_FALSE;
    if(fb->initialized == APP_FB_FALSE) return APP_FB_FALSE;

    fb->original_pid = controller->pid.param;
    fb->original_learning_enabled = controller->learning_enabled;
    fb->final_pid_valid = APP_FB_FALSE;
    fb->final_pid = fb->original_pid;

    /* AutoTune regression must evaluate PID only. Do not let adaptive FF
       learning move the plant input while candidates are being compared. */
    app_fb_temperature_controller_set_learning_enabled(controller, APP_FB_FALSE);

    app_fb_autotune_supervisor_start(&fb->supervisor, initial_pv);
    if(fb->supervisor.state == APP_FB_AUTOTUNE_SUPERVISOR_ERROR)
    {
        app_fb_autotune_binding_restore_learning(fb, controller);
        fb->status = APP_FB_AUTOTUNE_BINDING_ERROR;
        return APP_FB_FALSE;
    }

    fb->status = APP_FB_AUTOTUNE_BINDING_RUNNING;
    return APP_FB_TRUE;
}

APP_FB_AUTOTUNE_BINDING_STATUS_T app_fb_autotune_controller_binding_run(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    APP_FB_AUTOTUNE_SUPERVISOR_STATE_T state_before;
    APP_FB_AUTOTUNE_SUPERVISOR_STATE_T state_after;

    if(fb == NULL || controller == NULL || input == NULL || output == NULL)
        return APP_FB_AUTOTUNE_BINDING_ERROR;

    if(fb->status == APP_FB_AUTOTUNE_BINDING_IDLE)
    {
        app_fb_temperature_controller_run(controller, input, output);
        return fb->status;
    }

    if(fb->status == APP_FB_AUTOTUNE_BINDING_DONE)
    {
        app_fb_temperature_controller_run(controller, input, output);
        return fb->status;
    }

    if(fb->status == APP_FB_AUTOTUNE_BINDING_ERROR)
    {
        app_fb_autotune_binding_clear_output(output);
        return fb->status;
    }

    /* AutoTune validation is defined only for enabled AUTO operation. */
    if(input->enable == APP_FB_FALSE || input->mode != APP_FB_MODE_AUTO)
    {
        app_fb_autotune_supervisor_abort(&fb->supervisor);
        app_fb_temperature_controller_set_pid_parameter(
            controller,
            &fb->original_pid);
        app_fb_autotune_binding_restore_learning(fb, controller);
        app_fb_autotune_binding_clear_output(output);
        fb->status = APP_FB_AUTOTUNE_BINDING_ERROR;
        return fb->status;
    }

    state_before = fb->supervisor.state;

    if(state_before == APP_FB_AUTOTUNE_SUPERVISOR_RELAY)
    {
        state_after = app_fb_autotune_supervisor_run(
            &fb->supervisor,
            input->pv,
            0,
            &fb->supervisor_output);

        app_fb_autotune_binding_clear_output(output);
        output->error = input->sv - input->pv;

        if(fb->supervisor_output.relay_pwm_valid == APP_FB_TRUE)
        {
            output->pwm = APP_FB_LIMIT(
                fb->supervisor_output.relay_pwm,
                APP_FB_PWM_MIN,
                APP_FB_PWM_MAX);
        }

        if(fb->supervisor_output.candidate_changed == APP_FB_TRUE &&
           fb->supervisor_output.candidate_valid == APP_FB_TRUE)
        {
            app_fb_autotune_binding_apply_candidate(
                controller,
                input,
                &fb->supervisor_output.candidate_pid,
                output->pwm);
        }
    }
    else if(state_before == APP_FB_AUTOTUNE_SUPERVISOR_APPLY_CANDIDATE)
    {
        /* Candidate was loaded on the transition into APPLY_CANDIDATE. This
           supervisor call advances to REGRESSION without scoring this cycle. */
        state_after = app_fb_autotune_supervisor_run(
            &fb->supervisor,
            input->pv,
            controller->previous_pwm,
            &fb->supervisor_output);

        app_fb_temperature_controller_run(controller, input, output);
    }
    else if(state_before == APP_FB_AUTOTUNE_SUPERVISOR_REGRESSION)
    {
        /* Run the real controller first; regression must evaluate the actual
           post-limit PWM generated by this candidate. */
        app_fb_temperature_controller_run(controller, input, output);

        state_after = app_fb_autotune_supervisor_run(
            &fb->supervisor,
            input->pv,
            output->pwm,
            &fb->supervisor_output);

        if(fb->supervisor_output.candidate_changed == APP_FB_TRUE &&
           fb->supervisor_output.candidate_valid == APP_FB_TRUE)
        {
            app_fb_autotune_binding_apply_candidate(
                controller,
                input,
                &fb->supervisor_output.candidate_pid,
                output->pwm);
        }
    }
    else
    {
        state_after = app_fb_autotune_supervisor_run(
            &fb->supervisor,
            input->pv,
            controller->previous_pwm,
            &fb->supervisor_output);

        app_fb_temperature_controller_run(controller, input, output);
    }

    if(state_after == APP_FB_AUTOTUNE_SUPERVISOR_DONE)
    {
        if(app_fb_autotune_supervisor_get_final_pid(
               &fb->supervisor,
               &fb->final_pid) == APP_FB_TRUE)
        {
            /* The accepted candidate is already active. Do not reinitialize
               PID state here; keep the converged regression state bumpless. */
            fb->final_pid_valid = APP_FB_TRUE;
        }

        app_fb_autotune_binding_restore_learning(fb, controller);
        fb->status = APP_FB_AUTOTUNE_BINDING_DONE;
    }
    else if(state_after == APP_FB_AUTOTUNE_SUPERVISOR_ERROR)
    {
        /* Fail safe: restore known-good PID and learning configuration. */
        app_fb_autotune_binding_apply_candidate(
            controller,
            input,
            &fb->original_pid,
            output->pwm);
        app_fb_autotune_binding_restore_learning(fb, controller);
        fb->status = APP_FB_AUTOTUNE_BINDING_ERROR;
    }

    return fb->status;
}

void app_fb_autotune_controller_binding_abort(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    APP_FB_BOOL restore_original_pid)
{
    if(fb == NULL || controller == NULL) return;

    app_fb_autotune_supervisor_abort(&fb->supervisor);

    if(restore_original_pid != APP_FB_FALSE)
    {
        app_fb_temperature_controller_set_pid_parameter(
            controller,
            &fb->original_pid);
    }

    app_fb_autotune_binding_restore_learning(fb, controller);
    fb->final_pid_valid = APP_FB_FALSE;
    fb->status = APP_FB_AUTOTUNE_BINDING_IDLE;
}

APP_FB_BOOL app_fb_autotune_controller_binding_get_final_pid(
    const APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_PID_PARAMETER_T *pid)
{
    if(fb == NULL || pid == NULL) return APP_FB_FALSE;
    if(fb->status != APP_FB_AUTOTUNE_BINDING_DONE ||
       fb->final_pid_valid == APP_FB_FALSE)
    {
        return APP_FB_FALSE;
    }

    *pid = fb->final_pid;
    return APP_FB_TRUE;
}

#ifdef __cplusplus
}
#endif
