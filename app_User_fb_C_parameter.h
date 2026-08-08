/***************************************************************
Description : 
	This is a user APP_USER_C_PARAMETER program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

version: V0001

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_
#define SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_

//------------------------------------------------------------------------------------//
// C++ compatibility  // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//
// extern your program for others use.

#include "ssm_std_define.h"
#include "app_User_fb_C_control_type.h"

/*
====================================================
 Temperature Controller Parameter
====================================================
*/

/*
----------------------------------------------------
 Feedforward Table
----------------------------------------------------

Temperature:
0.1℃
PWM:
0.1%
Example:
175.0℃
=> 1750
----------------------------------------------------
*/

#define APP_FB_FF_TABLE_SIZE       (12)

/*
----------------------------------------------------
 PID Parameter
 Q15
 32768 = 1.0

 IMPORTANT:
 The PID implementation is a discrete-time controller.

 Ki is the discrete integral gain already including Ts:
     I(k) = I(k-1) + Error(k)
     Iout  = Ki * I(k)

 Kd is the discrete derivative gain for dPV per sample:
     D = -Kd * dPV(k)

 Therefore, when the controller sample time changes, Ki/Kd
 must be re-derived or re-tuned. Do not reuse continuous-time
 Ki/Kd values without conversion.
----------------------------------------------------
*/
typedef struct
{
    /* Proportional gain, Q15. */
    int32_t kp;

    /* Discrete integral gain, Q15; already includes Ts. */
    int32_t ki;

    /* Discrete derivative gain, Q15; dPV is per sample. */
    int32_t kd;

    /* Integral accumulator limit, in error-count samples. */
    int32_t integral_limit;

    /* PID correction output limit, PWM units. */
    int32_t output_limit;

    /* Back-calculation gain, Q15. */
    int32_t kaw;

} APP_FB_PID_PARAMETER_T;

/*
----------------------------------------------------
 Feedforward Parameter
----------------------------------------------------
*/
typedef struct
{
    APP_FB_TEMP temp[APP_FB_FF_TABLE_SIZE];
    APP_FB_PWM pwm[APP_FB_FF_TABLE_SIZE];

    /* Adaptive learning limit */
    int32_t learn_step;
    int32_t learn_limit;

} APP_FB_FF_PARAMETER_T;

/*
----------------------------------------------------
 Gain Scheduler
 Select by SV
----------------------------------------------------
*/

#define APP_FB_GAIN_ZONE_NUM       (3)

typedef struct
{
    APP_FB_TEMP low;
    APP_FB_TEMP high;
    APP_FB_PID_PARAMETER_T pid;

} APP_FB_GAIN_ZONE_T;

/*
----------------------------------------------------
 Integral Separation
 Unit:
 0.1℃
----------------------------------------------------
*/
typedef struct
{
    int32_t enable_error;

} APP_FB_INTEGRAL_SEPARATION_PARAMETER_T;

/*
----------------------------------------------------
 Derivative Filter
 Alpha Q15
 Df:
 y(k)=a*y(k-1)+(1-a)*x(k)
----------------------------------------------------
*/
typedef struct
{
    int32_t alpha;

} APP_FB_D_FILTER_PARAMETER_T;

/*
----------------------------------------------------
 Output Rate Limit
 PWM count / cycle
 20ms
----------------------------------------------------
*/
typedef struct
{
    int32_t rise_limit;
    int32_t fall_limit;

} APP_FB_RATE_LIMIT_PARAMETER_T;

/*
----------------------------------------------------
 Anti Windup
 Back Calculation
----------------------------------------------------
*/
typedef struct
{
    int32_t kaw;

} APP_FB_ANTI_WINDUP_PARAMETER_T;

/*
----------------------------------------------------
 Adaptive Feedforward
 Update every 1 sec
----------------------------------------------------
*/
typedef struct
{
    /* Error threshold, 0.1℃ */
    int32_t error_threshold;

    /* learning gain, Q15 */
    int32_t gain;

} APP_FB_ADAPTIVE_PARAMETER_T;

/*
====================================================
 Default Parameter
====================================================
*/

#define APP_FB_PID_KP_DEFAULT          32768
#define APP_FB_PID_KI_DEFAULT          600
#define APP_FB_PID_KD_DEFAULT          65536
#define APP_FB_PID_INTEGRAL_LIMIT      3000
#define APP_FB_PID_OUTPUT_LIMIT        800
#define APP_FB_PID_KAW_DEFAULT         8192

/* Integral Separation: 2℃ */
#define APP_FB_I_ENABLE_ERROR          20

/* Derivative LPF: alpha = 0.875 */
#define APP_FB_D_FILTER_ALPHA          28672

/* PWM Rate Limit: per 20ms */
#define APP_FB_PWM_RISE_LIMIT          30
#define APP_FB_PWM_FALL_LIMIT          50

/* Anti Windup: Kaw = 0.25 */
#define APP_FB_KAW                     8192

/* Adaptive */
#define APP_FB_ADAPTIVE_ERROR          10
#define APP_FB_ADAPTIVE_GAIN           512

//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_
