/***************************************************************
Description :
    Application wrapper API for adaptive temperature controller.
***************************************************************/
#ifndef ENTRY_C_ADPTIVE_TEMP_CONTROLLER_H_
#define ENTRY_C_ADPTIVE_TEMP_CONTROLLER_H_

#include <stdint.h>
#include "ssm_std_define.h"
#include "FB_C_adaptive_temperature_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    APP_FB_BOOL self_tuning_enabled;
    APP_FB_BOOL response_active;
    APP_FB_BOOL settled;
    APP_FB_BOOL response_qualified;
    int32_t current_ki;
    uint32_t current_predictive_time_ms;
    APP_FB_TEMP peak_pv;
    APP_FB_TEMP overshoot;
    APP_FB_TEMP steady_error;
    int32_t pv_slope_per_s;
    uint32_t elapsed_time_ms;
    uint32_t settling_time_ms;
    uint32_t cooldown_remaining_ms;
    uint32_t tune_count;
    uint32_t max_tunes_per_session;
    APP_FB_SELF_TUNE_REASON_T last_tune_reason;
} HEATER_SELF_TUNING_DIAGNOSTICS_T;

MY_API void Heater_Control_Init(void);
MY_API void Heater_Control_InitEx(const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);
MY_API APP_FB_ERROR Heater_Control_InitTimed(uint32_t sample_time_ms);
MY_API APP_FB_ERROR Heater_Control_InitExTimed(const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter,uint32_t sample_time_ms);
MY_API uint32_t Heater_GetSampleTimeMs(void);
MY_API void Heater_SetAdaptiveParameter(const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);

/* Self tuning is OFF by default. Diagnostics remain active in AUTO mode. */
MY_API void Heater_SetSelfTuningEnable(APP_FB_BOOL enable);
MY_API APP_FB_BOOL Heater_GetSelfTuningEnable(void);
MY_API APP_FB_ERROR Heater_GetSelfTuningDiagnostics(HEATER_SELF_TUNING_DIAGNOSTICS_T *diagnostics);

MY_API void Heater_myAdptiveControl(
    int16_t input_pv,
    int16_t input_sv,
    int32_t *output_pid_out,
    int32_t *output_ff_pwm,
    int32_t *output_ff_offset,
    int32_t *output_heater_pwm);

#ifdef __cplusplus
}
#endif
#endif
