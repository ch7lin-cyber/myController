/***************************************************************
Description : 
	This is a user USER_ADAPTIVE_TEMP_CONTROLLER program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_
#define SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_


#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"
#include "app_User_fb_C_pid.h"

#include "app_User_fb_C_feedforward_table.h"

#include "app_User_fb_C_derivative_filter.h"
#include "app_User_fb_C_integral_separation.h"
#include "app_User_fb_C_output_rate_limit.h"
#include "app_User_fb_C_feedforward_learning.h"


//------------------------------------------------------------------------------------//
// C++ compatibility  // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//
// extern your program for others use.

/*
====================================================
 Temperature Controller Input
====================================================
*/

typedef struct
{

    /*
     * Set Point
     * 0.1℃
     */
    APP_FB_TEMP sv;

    /*
     * Process Value
     * 0.1℃
     */
    APP_FB_TEMP pv;

    /*
     * Enable
     */
    APP_FB_BOOL enable;

    /*
     * Manual PWM
     */
    APP_FB_PWM manual_pwm;

    /*
     * Mode
     */
    APP_FB_CONTROL_MODE mode;

}APP_FB_TEMP_CONTROLLER_INPUT_T;


/*
====================================================
 Output
====================================================
*/
typedef struct
{
    /*
     * PWM Output
     * 0~1000
     */
    APP_FB_PWM pwm;

    /*
     * Feedforward value
     */
    APP_FB_PWM ff_pwm;

    /*
     * PID correction
     */
    int32_t pid_output;

    /*
     * Learning offset
     */
    int32_t ff_offset;

    /*
     * Error
     */
    int32_t error;

}APP_FB_TEMP_CONTROLLER_OUTPUT_T;


/*
====================================================
 Function Block Instance
====================================================
*/
typedef struct
{
    /*
     * PID
     */
    APP_FB_PID_T  pid;

    /*
     * Feedforward
     */
    APP_FB_FEEDFORWARD_T ff;

    /*
     * Derivative filter
     */
    APP_FB_D_FILTER_T d_filter;

    /*
     * Integral Separation
     */
    APP_FB_INTEGRAL_SEPARATION_T i_sep;

    /*
     * PWM Rate Limit
     */
    APP_FB_RATE_LIMIT_T rate_limit;

    /*
     * Adaptive FF Learning
     */
    APP_FB_FF_LEARNING_T learning;

    /*
     * Previous PWM
     */
    int32_t previous_pwm;

    /*
     * Running State
     */
    APP_FB_STATE state;

}APP_FB_TEMPERATURE_CONTROLLER_T;


/*
====================================================
 Initialize

====================================================
*/
MY_API void app_fb_temperature_controller_init
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_FF_POINT_T *ff_table,
    int32_t ff_size,
    const APP_FB_PID_PARAMETER_T *pid_parameter
);

/*
====================================================
 Execute 50Hz
====================================================
*/
MY_API void app_fb_temperature_controller_run
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output
);


/*
====================================================
 Reset

====================================================
*/
MY_API void app_fb_temperature_controller_reset
(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb
);

MY_API void Heater_Control_Init(void);
MY_API void Heater_myAdptiveControl(int16_t input_pv, int16_t input_sv, int32_t *output_pid_out, int32_t *output_ff_pwm, int32_t *output_ff_offset);




//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_ADAPTIVE_TEMP_CONTROLLER_CODE_H_




