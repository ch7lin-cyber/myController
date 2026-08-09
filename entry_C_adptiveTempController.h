/***************************************************************
Description :
    Application wrapper API for adaptive temperature controller.

Purpose:
    Keep platform/application entry points separate from the reusable
    PID_ControllerSrc library interface.
***************************************************************/
#ifndef ENTRY_C_ADPTIVE_TEMP_CONTROLLER_H_
#define ENTRY_C_ADPTIVE_TEMP_CONTROLLER_H_

#include <stdint.h>
#include "ssm_std_define.h"
#include "PID_ControllerSrc/FB_C_adaptive_temperature_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Backward-compatible initialization using default adaptive parameters. */
MY_API void Heater_Control_Init(void);

/* Extended initialization with runtime adaptive-learning parameters. */
MY_API void Heater_Control_InitEx(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);

MY_API void Heater_myAdptiveControl(
    int16_t input_pv,
    int16_t input_sv,
    int32_t *output_pid_out,
    int32_t *output_ff_pwm,
    int32_t *output_ff_offset);

#ifdef __cplusplus
}
#endif

#endif
