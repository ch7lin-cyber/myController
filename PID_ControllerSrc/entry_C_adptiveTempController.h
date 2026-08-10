/***************************************************************
Description :
    Application wrapper API for adaptive temperature controller.

Purpose:
    Keep platform/application entry points together with the reusable
    PID_ControllerSrc library sources.

Timing policy:
    The application/scheduler owns the execution period. The period is passed
    once during initialization and remains fixed while the controller runs.
***************************************************************/
#ifndef ENTRY_C_ADPTIVE_TEMP_CONTROLLER_H_
#define ENTRY_C_ADPTIVE_TEMP_CONTROLLER_H_

#include <stdint.h>
#include "ssm_std_define.h"
#include "FB_C_adaptive_temperature_controller.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

/* Legacy/default initialization retained for source compatibility. */
MY_API void Heater_Control_Init(void);

MY_API void Heater_Control_InitEx(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);

/* Preferred initialization-only timing APIs. */
MY_API APP_FB_ERROR Heater_Control_InitTimed(
    uint32_t sample_time_ms);

MY_API APP_FB_ERROR Heater_Control_InitExTimed(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter,
    uint32_t sample_time_ms);

MY_API uint32_t Heater_GetSampleTimeMs(void);

MY_API void Heater_SetAdaptiveParameter(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);

MY_API void Heater_myAdptiveControl(
    int16_t input_pv,
    int16_t input_sv,
    int32_t *output_pid_out,
    int32_t *output_ff_pwm,
    int32_t *output_ff_offset);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
