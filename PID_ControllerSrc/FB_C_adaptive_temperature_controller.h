/***************************************************************
Description :
    User adaptive temperature controller application header.

Change notice:
[ADD] MANUAL -> AUTO bumpless-transfer state.
***************************************************************/
#ifndef SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_
#define SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_

#include "FB_C_control_type.h"
#include "FB_C_parameter.h"
#include "FB_C_pid.h"
#include "FB_C_feedforward_table.h"
#include "FB_C_derivative_filter.h"
#include "FB_C_integral_separation.h"
#include "FB_C_output_rate_limit.h"
#include "FB_C_feedforward_learning.h"

#ifdef __cplusplus
extern "C" {
#endif

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
    int32_t previous_pwm;
    APP_FB_STATE state;
    APP_FB_BOOL manual_active;
} APP_FB_TEMPERATURE_CONTROLLER_T;

MY_API void app_fb_temperature_controller_init(APP_FB_TEMPERATURE_CONTROLLER_T *fb, const APP_FB_FF_POINT_T *ff_table, int32_t ff_size, const APP_FB_PID_PARAMETER_T *pid_parameter);
MY_API void app_fb_temperature_controller_run(APP_FB_TEMPERATURE_CONTROLLER_T *fb, const APP_FB_TEMP_CONTROLLER_INPUT_T *input, APP_FB_TEMP_CONTROLLER_OUTPUT_T *output);
MY_API void app_fb_temperature_controller_reset(APP_FB_TEMPERATURE_CONTROLLER_T *fb);
MY_API void Heater_Control_Init(void);
MY_API void Heater_myAdptiveControl(int16_t input_pv, int16_t input_sv, int32_t *output_pid_out, int32_t *output_ff_pwm, int32_t *output_ff_offset);

#ifdef __cplusplus
}
#endif

#endif
