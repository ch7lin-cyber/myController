#ifndef FB_C_AUTOTUNE_CONTROLLER_BINDING_H_
#define FB_C_AUTOTUNE_CONTROLLER_BINDING_H_

#include <stdint.h>
#include "../PID_ControllerSrc/FB_C_adaptive_temperature_controller.h"
#include "FB_C_autotune_supervisor.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef enum
{
    APP_FB_AUTOTUNE_BINDING_IDLE = 0,
    APP_FB_AUTOTUNE_BINDING_RUNNING,
    APP_FB_AUTOTUNE_BINDING_DONE,
    APP_FB_AUTOTUNE_BINDING_ERROR
} APP_FB_AUTOTUNE_BINDING_STATUS_T;

typedef struct
{
    APP_FB_AUTOTUNE_SUPERVISOR_T supervisor;
    APP_FB_AUTOTUNE_SUPERVISOR_OUTPUT_T supervisor_output;

    APP_FB_PID_PARAMETER_T original_pid;
    APP_FB_PID_PARAMETER_T final_pid;

    APP_FB_BOOL original_learning_enabled;
    APP_FB_BOOL final_pid_valid;
    APP_FB_BOOL initialized;

    APP_FB_AUTOTUNE_BINDING_STATUS_T status;
} APP_FB_AUTOTUNE_CONTROLLER_BINDING_T;

void app_fb_autotune_controller_binding_init(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    const APP_FB_AUTOTUNE_SUPERVISOR_CONFIG_T *cfg);

APP_FB_BOOL app_fb_autotune_controller_binding_start(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    APP_FB_TEMP initial_pv);

/*
 * Execute once per normal controller cycle.
 *
 * RELAY phase:
 *   Normal PID is bypassed and output->pwm is commanded by the relay tuner.
 *
 * APPLY/REGRESSION phase:
 *   Candidate PID is loaded through the controller runtime API. The normal
 *   FF + PID controller then runs and its real PWM is evaluated.
 *
 * Adaptive Learning is disabled for the complete AutoTune session and is
 * restored automatically on DONE / ERROR / ABORT.
 */
APP_FB_AUTOTUNE_BINDING_STATUS_T app_fb_autotune_controller_binding_run(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output);

void app_fb_autotune_controller_binding_abort(
    APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_TEMPERATURE_CONTROLLER_T *controller,
    APP_FB_BOOL restore_original_pid);

APP_FB_BOOL app_fb_autotune_controller_binding_get_final_pid(
    const APP_FB_AUTOTUNE_CONTROLLER_BINDING_T *fb,
    APP_FB_PID_PARAMETER_T *pid);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* FB_C_AUTOTUNE_CONTROLLER_BINDING_H_ */
