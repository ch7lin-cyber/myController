#include "FB_C_autotune_supervisor.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

static APP_FB_BOOL app_fb_autotune_supervisor_config_valid(
    const APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T *cfg)
{
    if(cfg == NULL) return APP_FB_FALSE;

    /* Relay and regression must evaluate the same operating point. */
    if(cfg->relay.sv != cfg->regression.sv) return APP_FB_FALSE;

    if(cfg->relay.sample_time_ms == 0U ||
       cfg->regression.sample_time_ms == 0U)
    {
        return APP_FB_FALSE;
    }

    if(cfg->relay.sample_time_ms != cfg->regression.sample_time_ms)
        return APP_FB_FALSE;

    if(cfg->regression.pwm_min < cfg->relay.pwm_min ||
       cfg->regression.pwm_max > cfg->relay.pwm_max)
    {
        return APP_FB_FALSE;
    }

    return APP_FB_TRUE;
}

static void app_fb_autotune_supervisor_zero_pid(APP_FB_PID_PARAMETER_T *pid)
{
    if(pid == NULL) return;
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;
    pid->integral_limit = 0;
    pid->output_limit = 0;
    pid->kaw = 0;
}

static void app_fb_autotune_supervisor_clear_runtime(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb)
{
    if(fb == NULL) return;

    fb->candidate_valid = APP_FB_FALSE;
    fb->final_pid_valid = APP_FB_FALSE;
    fb->last_reject_reason = APP_FB_AUTOTUNE_REJECT_NONE;

    app_fb_autotune_supervisor_zero_pid(&fb->candidate_pid);
    app_fb_autotune_supervisor_zero_pid(&fb->final_pid);

    fb->gain_guard.current_scale_percent = 0U;
    fb->gain_guard.attempt_count = 0U;
    fb->gain_guard.status = APP_FB_AUTOTUNE_VALIDATION_FAIL;
    fb->gain_guard.reject_reason = APP_FB_AUTOTUNE_REJECT_NONE;
}

static void app_fb_autotune_supervisor_clear_output(
    APP_FB_AUTOTUNE_SUPERVISOR_OUTPUT_T *output)
{
    if(output == NULL) return;

    output->state = APP_FB_AUTOTUNE_SUPERVISOR_IDLE;
    output->relay_pwm_valid = APP_FB_FALSE;
    output->relay_pwm = 0;
    output->candidate_valid = APP_FB_FALSE;
    output->candidate_changed = APP_FB_FALSE;
    app_fb_autotune_supervisor_zero_pid(&output->candidate_pid);
    output->final_pid_valid = APP_FB_FALSE;
    app_fb_autotune_supervisor_zero_pid(&output->final_pid);
    output->gain_scale_percent = 0U;
    output->attempt_count = 0U;
    output->last_reject_reason = APP_FB_AUTOTUNE_REJECT_NONE;
}

static void app_fb_autotune_supervisor_publish(
    const APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_AUTOTUNE_SUPERVISOR_OUTPUT_T *output,
    APP_FB_BOOL candidate_changed)
{
    if(fb == NULL || output == NULL) return;

    output->state = fb->state;
    output->candidate_valid = fb->candidate_valid;
    output->candidate_changed = candidate_changed;
    output->candidate_pid = fb->candidate_pid;
    output->final_pid_valid = fb->final_pid_valid;
    output->final_pid = fb->final_pid;
    output->gain_scale_percent = fb->gain_guard.current_scale_percent;
    output->attempt_count = fb->gain_guard.attempt_count;
    output->last_reject_reason = fb->last_reject_reason;
}

void app_fb_autotune_supervisor_init(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    const APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T *cfg)
{
    if(fb == NULL) return;

    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_IDLE;
    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NONE;
    fb->initialized = APP_FB_FALSE;
    app_fb_autotune_supervisor_clear_runtime(fb);

    if(cfg == NULL)
    {
        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
        fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NULL_POINTER;
        return;
    }

    if(app_fb_autotune_supervisor_config_valid(cfg) == APP_FB_FALSE)
    {
        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
        fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_PARAMETER;
        return;
    }

    fb->cfg = *cfg;
    app_fb_relay_autotune_init(&fb->relay, &fb->cfg.relay);

    if(app_fb_relay_autotune_get_status(&fb->relay) == APP_FB_AUTOTUNE_ERROR)
    {
        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
        fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_RELAY;
        return;
    }

    fb->initialized = APP_FB_TRUE;
}

void app_fb_autotune_supervisor_start(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_TEMP initial_pv)
{
    if(fb == NULL) return;
    if(fb->initialized == APP_FB_FALSE)
    {
        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
        fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_PARAMETER;
        return;
    }

    app_fb_autotune_supervisor_clear_runtime(fb);
    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NONE;

    app_fb_relay_autotune_init(&fb->relay, &fb->cfg.relay);
    app_fb_relay_autotune_start(&fb->relay, initial_pv);

    if(app_fb_relay_autotune_get_status(&fb->relay) == APP_FB_AUTOTUNE_ERROR)
    {
        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
        fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_RELAY;
        return;
    }

    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_RELAY;
}

APP_FB_AUTOTUNE_SUPERVISOR_STATE_T app_fb_autotune_supervisor_run(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_TEMP pv,
    APP_FB_PWM controller_pwm,
    APP_FB_AUTOTUNE_SUPERVISOR_OUTPUT_T *output)
{
    APP_FB_AUTOTUNE_STATUS_T relay_status;
    APP_FB_AUTOTUNE_REGRESSION_STATUS_T regression_status;
    APP_FB_AUTOTUNE_REGRESSION_METRICS_T metrics;
    APP_FB_AUTOTUNE_VALIDATION_STATUS_T validation_status;
    APP_FB_BOOL candidate_changed;

    if(output != NULL) app_fb_autotune_supervisor_clear_output(output);

    if(fb == NULL || output == NULL)
        return APP_FB_AUTOTUNE_SUPERVISOR_ERROR;

    candidate_changed = APP_FB_FALSE;

    switch(fb->state)
    {
        case APP_FB_AUTOTUNE_SUPERVISOR_IDLE:
            break;

        case APP_FB_AUTOTUNE_SUPERVISOR_RELAY:
            output->relay_pwm = app_fb_relay_autotune_run(&fb->relay, pv);
            output->relay_pwm_valid = APP_FB_TRUE;
            relay_status = app_fb_relay_autotune_get_status(&fb->relay);

            if(relay_status == APP_FB_AUTOTUNE_DONE)
            {
                if(app_fb_relay_autotune_get_result(&fb->relay, &fb->relay_result) == APP_FB_FALSE)
                {
                    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_RELAY;
                    break;
                }

                app_fb_autotune_gain_guard_init(
                    &fb->gain_guard,
                    &fb->cfg.gain_guard,
                    &fb->relay_result.pid);

                if(app_fb_autotune_gain_guard_get_candidate(
                       &fb->gain_guard,
                       &fb->candidate_pid) == APP_FB_FALSE)
                {
                    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_GAIN_GUARD;
                    fb->last_reject_reason =
                        app_fb_autotune_gain_guard_get_reject_reason(&fb->gain_guard);
                    break;
                }

                fb->candidate_valid = APP_FB_TRUE;
                fb->state = APP_FB_AUTOTUNE_SUPERVISOR_APPLY_CANDIDATE;
                candidate_changed = APP_FB_TRUE;
            }
            else if(relay_status == APP_FB_AUTOTUNE_ERROR)
            {
                fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_RELAY;
            }
            break;

        case APP_FB_AUTOTUNE_SUPERVISOR_APPLY_CANDIDATE:
            /*
             * Candidate was published on the previous cycle. The application
             * has one full cycle to load candidate_pid before regression is
             * armed here. No regression sample is consumed in this state.
             */
            app_fb_autotune_regression_init(
                &fb->regression,
                &fb->cfg.regression,
                pv);

            if(fb->regression.status == APP_FB_AUTOTUNE_REGRESSION_ERROR)
            {
                fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_REGRESSION;
            }
            else
            {
                fb->state = APP_FB_AUTOTUNE_SUPERVISOR_REGRESSION;
            }
            break;

        case APP_FB_AUTOTUNE_SUPERVISOR_REGRESSION:
            regression_status = app_fb_autotune_regression_run(
                &fb->regression,
                pv,
                controller_pwm);

            if(regression_status == APP_FB_AUTOTUNE_REGRESSION_DONE ||
               regression_status == APP_FB_AUTOTUNE_REGRESSION_TIMEOUT)
            {
                if(app_fb_autotune_regression_get_metrics(
                       &fb->regression,
                       &metrics) == APP_FB_FALSE)
                {
                    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_REGRESSION;
                    break;
                }

                validation_status = app_fb_autotune_gain_guard_evaluate(
                    &fb->gain_guard,
                    &metrics);

                fb->last_reject_reason =
                    app_fb_autotune_gain_guard_get_reject_reason(&fb->gain_guard);

                if(validation_status == APP_FB_AUTOTUNE_VALIDATION_PASS)
                {
                    fb->final_pid = fb->candidate_pid;
                    fb->final_pid_valid = APP_FB_TRUE;
                    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_DONE;
                }
                else if(validation_status == APP_FB_AUTOTUNE_VALIDATION_RETRY)
                {
                    if(app_fb_autotune_gain_guard_get_candidate(
                           &fb->gain_guard,
                           &fb->candidate_pid) == APP_FB_FALSE)
                    {
                        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                        fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_GAIN_GUARD;
                    }
                    else
                    {
                        fb->candidate_valid = APP_FB_TRUE;
                        fb->state = APP_FB_AUTOTUNE_SUPERVISOR_APPLY_CANDIDATE;
                        candidate_changed = APP_FB_TRUE;
                    }
                }
                else
                {
                    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_GAIN_GUARD;
                }
            }
            else if(regression_status == APP_FB_AUTOTUNE_REGRESSION_ERROR)
            {
                fb->state = APP_FB_AUTOTUNE_SUPERVISOR_ERROR;
                fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_REGRESSION;
            }
            break;

        case APP_FB_AUTOTUNE_SUPERVISOR_DONE:
            break;

        case APP_FB_AUTOTUNE_SUPERVISOR_ERROR:
        default:
            break;
    }

    app_fb_autotune_supervisor_publish(fb, output, candidate_changed);

    if(fb->state == APP_FB_AUTOTUNE_SUPERVISOR_RELAY)
    {
        output->relay_pwm_valid = APP_FB_TRUE;
        output->relay_pwm = fb->relay.output_pwm;
    }

    return fb->state;
}

void app_fb_autotune_supervisor_abort(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb)
{
    if(fb == NULL) return;

    app_fb_relay_autotune_abort(&fb->relay);
    fb->state = APP_FB_AUTOTUNE_SUPERVISOR_IDLE;
    fb->error = APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NONE;
    app_fb_autotune_supervisor_clear_runtime(fb);
}

APP_FB_AUTOTUNE_SUPERVISOR_ERROR_T app_fb_autotune_supervisor_get_error(
    const APP_FB_AUTOTUNE_SUPERVISOR_T *fb)
{
    if(fb == NULL) return APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NULL_POINTER;
    return fb->error;
}

APP_FB_BOOL app_fb_autotune_supervisor_get_final_pid(
    const APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_PID_PARAMETER_T *pid)
{
    if(fb == NULL || pid == NULL) return APP_FB_FALSE;
    if(fb->state != APP_FB_AUTOTUNE_SUPERVISOR_DONE ||
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
