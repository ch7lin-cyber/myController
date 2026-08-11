/***************************************************************
Description :
    User adaptive temperature controller application header.

Change notice:
[ADD] MANUAL -> AUTO bumpless-transfer state.
[MODIFY] Remove application wrapper API declarations from library header.
[ADD] Runtime adaptive-learning parameter wiring via init_ex().
[ADD] Runtime adaptive-learning reconfiguration API.
[ADD] Load-disturbance integral arming and SV-change tracking.
[ADD] Initialization-only sample-time ownership from outer application layer.
[ADD] Fast Heating Boost V3 with predictive thermal braking.
***************************************************************/
#ifndef SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_
#define SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"
#include "FB_C_pid.h"
#include "FB_C_feedforward_table.h"
#include "FB_C_derivative_filter.h"
#include "FB_C_integral_separation.h"
#include "FB_C_output_rate_limit.h"
#include "FB_C_feedforward_learning.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef struct
{
    APP_FB_TEMP sv;
    APP_FB_TEMP pv;
    APP_FB_BOOL enable;
    APP_FB_PWM manual_pwm;
    APP_FB_CONTROL_MODE mode;
} APP_FB_TEMP_CONTROLLER_INPUT_T;

typedef struct
{
    APP_FB_PWM pwm;
    APP_FB_PWM ff_pwm;
    int32_t pid_output;
    int32_t ff_offset;
    int32_t error;
} APP_FB_TEMP_CONTROLLER_OUTPUT_T;

typedef struct
{
    APP_FB_PID_T pid;
    APP_FB_FEEDFORWARD_T ff;
    APP_FB_D_FILTER_T d_filter;
    APP_FB_INTEGRAL_SEPARATION_T i_sep;
    APP_FB_RATE_LIMIT_T rate_limit;
    APP_FB_FF_LEARNING_T learning;

    APP_FB_TIMING_PARAMETER_T timing;
    uint32_t sample_time_us;

    int32_t previous_pwm;
    APP_FB_TEMP previous_sv;
    APP_FB_STATE state;
    APP_FB_BOOL manual_active;
    APP_FB_BOOL sv_initialized;
    APP_FB_BOOL integral_disturbance_armed;
    APP_FB_BOOL fast_heat_active;
    APP_FB_BOOL predictive_brake_active;
} APP_FB_TEMPERATURE_CONTROLLER_T;

/* Legacy init: uses APP_FB_SAMPLE_TIME_DEFAULT_MS. */
MY_API void app_fb_temperature_controller_init(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter);

/* Legacy init_ex: uses APP_FB_SAMPLE_TIME_DEFAULT_MS. */
MY_API void app_fb_temperature_controller_init_ex(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);

/*
 * Timing-aware initialization. sample_time_ms is supplied by the outer
 * scheduler/application and remains fixed for the lifetime of this init.
 */
MY_API APP_FB_ERROR app_fb_temperature_controller_init_timed(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_TIMING_PARAMETER_T *timing_parameter);

MY_API APP_FB_ERROR app_fb_temperature_controller_init_ex_timed(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter,
    const APP_FB_TIMING_PARAMETER_T *timing_parameter);

/* Read-only timing diagnostics after initialization. */
MY_API uint32_t app_fb_temperature_controller_get_sample_time_ms(
    const APP_FB_TEMPERATURE_CONTROLLER_T *fb);

MY_API uint32_t app_fb_temperature_controller_get_sample_time_us(
    const APP_FB_TEMPERATURE_CONTROLLER_T *fb);

MY_API void app_fb_temperature_controller_set_adaptive_parameter(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter);

MY_API void app_fb_temperature_controller_run(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output);

MY_API void app_fb_temperature_controller_reset(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
