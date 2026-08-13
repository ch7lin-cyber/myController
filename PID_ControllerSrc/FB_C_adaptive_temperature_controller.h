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
#include "FB_C_process_observer.h"
#include "FB_C_self_tuner.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct{APP_FB_TEMP sv,pv;APP_FB_BOOL enable;APP_FB_PWM manual_pwm;APP_FB_CONTROL_MODE mode;}APP_FB_TEMP_CONTROLLER_INPUT_T;
typedef struct{APP_FB_PWM pwm,ff_pwm;int32_t pid_output,ff_offset,error;}APP_FB_TEMP_CONTROLLER_OUTPUT_T;
typedef struct{APP_FB_PID_T pid;APP_FB_FEEDFORWARD_T ff;APP_FB_D_FILTER_T d_filter;APP_FB_INTEGRAL_SEPARATION_T i_sep;APP_FB_RATE_LIMIT_T rate_limit;APP_FB_FF_LEARNING_T learning;APP_FB_PROCESS_OBSERVER_T observer;APP_FB_SELF_TUNER_T self_tuner;APP_FB_TIMING_PARAMETER_T timing;uint32_t sample_time_us,predictive_brake_time_ms;int32_t previous_pwm;APP_FB_TEMP previous_sv;APP_FB_STATE state;APP_FB_BOOL manual_active,sv_initialized,integral_disturbance_armed,fast_heat_active,predictive_brake_active;}APP_FB_TEMPERATURE_CONTROLLER_T;
MY_API void app_fb_temperature_controller_init(APP_FB_TEMPERATURE_CONTROLLER_T*,const APP_FB_FF_POINT_T*,int32_t,const APP_FB_PID_PARAMETER_T*);
MY_API void app_fb_temperature_controller_init_ex(APP_FB_TEMPERATURE_CONTROLLER_T*,const APP_FB_FF_POINT_T*,int32_t,const APP_FB_PID_PARAMETER_T*,const APP_FB_ADAPTIVE_PARAMETER_T*);
MY_API APP_FB_ERROR app_fb_temperature_controller_init_timed(APP_FB_TEMPERATURE_CONTROLLER_T*,const APP_FB_FF_POINT_T*,int32_t,const APP_FB_PID_PARAMETER_T*,const APP_FB_TIMING_PARAMETER_T*);
MY_API APP_FB_ERROR app_fb_temperature_controller_init_ex_timed(APP_FB_TEMPERATURE_CONTROLLER_T*,const APP_FB_FF_POINT_T*,int32_t,const APP_FB_PID_PARAMETER_T*,const APP_FB_ADAPTIVE_PARAMETER_T*,const APP_FB_TIMING_PARAMETER_T*);
MY_API uint32_t app_fb_temperature_controller_get_sample_time_ms(const APP_FB_TEMPERATURE_CONTROLLER_T*);
MY_API uint32_t app_fb_temperature_controller_get_sample_time_us(const APP_FB_TEMPERATURE_CONTROLLER_T*);
MY_API void app_fb_temperature_controller_set_adaptive_parameter(APP_FB_TEMPERATURE_CONTROLLER_T*,const APP_FB_ADAPTIVE_PARAMETER_T*);
MY_API void app_fb_temperature_controller_run(APP_FB_TEMPERATURE_CONTROLLER_T*,const APP_FB_TEMP_CONTROLLER_INPUT_T*,APP_FB_TEMP_CONTROLLER_OUTPUT_T*);
MY_API void app_fb_temperature_controller_reset(APP_FB_TEMPERATURE_CONTROLLER_T*);
#ifdef __cplusplus
}
#endif
#endif
