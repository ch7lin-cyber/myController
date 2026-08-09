#ifndef FB_C_AUTOTUNE_SUPERVISOR_H_
#define FB_C_AUTOTUNE_SUPERVISOR_H_

#include <stdint.h>
#include "../PID_ControllerSrc/FB_C_control_type.h"
#include "../PID_ControllerSrc/FB_C_parameter.h"
#include "FB_C_relay_auto_tune.h"
#include "FB_C_autotune_gain_guard.h"
#include "FB_C_autotune_regression_runner.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef enum
{
    APP_FB_AUTOTUNE_SUPERVISOR_IDLE = 0,
    APP_FB_AUTOTUNE_SUPERVISOR_RELAY,
    APP_FB_AUTOTUNE_SUPERVISOR_APPLY_CANDIDATE,
    APP_FB_AUTOTUNE_SUPERVISOR_REGRESSION,
    APP_FB_AUTOTUNE_SUPERVISOR_DONE,
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR
} APP_FB_AUTOTUNE_SUPERVISOR_STATE_T;

typedef enum
{
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NONE = 0,
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_NULL_POINTER,
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_PARAMETER,
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_RELAY,
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_GAIN_GUARD,
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_REGRESSION
} APP_FB_AUTOTUNE_SUPERVISOR_ERROR_T;

typedef struct
{
    APP_FB_RELAY_AUTOTUNE_CONFIG_T relay;
    APP_FB_AUTOTUNE_GAIN_GUARD_CONFIG_T gain_guard;
    APP_FB_AUTOTUNE_REGRESSION_CONFIG_T regression;
} APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T;

typedef struct
{
    APP_FB_AUTOTUNE_SUPERVISOR_STATE_T state;

    /* Relay phase: bypass normal controller and apply this PWM directly. */
    APP_FB_BOOL relay_pwm_valid;
    APP_FB_PWM relay_pwm;

    /* Apply-candidate phase: load this PID into the normal controller. */
    APP_FB_BOOL candidate_valid;
    APP_FB_BOOL candidate_changed;
    APP_FB_PID_PARAMETER_T candidate_pid;

    /* DONE phase: accepted PID after regression. */
    APP_FB_BOOL final_pid_valid;
    APP_FB_PID_PARAMETER_T final_pid;

    uint16_t gain_scale_percent;
    uint16_t attempt_count;

    APP_FB_AUTOTUNE_REJECT_REASON_T last_reject_reason;
} APP_FB_AUTOTUNE_SUPERVISOR_OUTPUT_T;

typedef struct
{
    APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T cfg;

    APP_FB_RELAY_AUTOTUNE_T relay;
    APP_FB_AUTOTUNE_GAIN_GUARD_T gain_guard;
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T regression;

    APP_FB_RELAY_AUTOTUNE_RESULT_T relay_result;
    APP_FB_PID_PARAMETER_T candidate_pid;
    APP_FB_PID_PARAMETER_T final_pid;

    APP_FB_AUTOTUNE_SUPERVISOR_STATE_T state;
    APP_FB_AUTOTUNE_SUPERVISOR_ERROR_T error;
    APP_FB_AUTOTUNE_REJECT_REASON_T last_reject_reason;

    APP_FB_BOOL initialized;
    APP_FB_BOOL candidate_valid;
    APP_FB_BOOL final_pid_valid;
} APP_FB_AUTOTUNE_SUPERVISOR_T;

void app_fb_autotune_supervisor_init(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    const APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T *cfg);

void app_fb_autotune_supervisor_start(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_TEMP initial_pv);

/*
 * Execute once per control cycle.
 *
 * pv:
 *   Current measured process value.
 *
 * controller_pwm:
 *   Actual PWM produced by the normal controller while in REGRESSION state.
 *   It is ignored during RELAY / APPLY_CANDIDATE states.
 *
 * During APPLY_CANDIDATE, the caller must load output.candidate_pid into its
 * controller. On the next cycle, Supervisor enters REGRESSION and begins
 * evaluating the real SV/PV/PWM stream.
 */
APP_FB_AUTOTUNE_SUPERVISOR_STATE_T app_fb_autotune_supervisor_run(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_TEMP pv,
    APP_FB_PWM controller_pwm,
    APP_FB_AUTOTUNE_SUPERVISOR_OUTPUT_T *output);

void app_fb_autotune_supervisor_abort(
    APP_FB_AUTOTUNE_SUPERVISOR_T *fb);

APP_FB_AUTOTUNE_SUPERVISOR_ERROR_T app_fb_autotune_supervisor_get_error(
    const APP_FB_AUTOTUNE_SUPERVISOR_T *fb);

APP_FB_BOOL app_fb_autotune_supervisor_get_final_pid(
    const APP_FB_AUTOTUNE_SUPERVISOR_T *fb,
    APP_FB_PID_PARAMETER_T *pid);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* FB_C_AUTOTUNE_SUPERVISOR_H_ */
