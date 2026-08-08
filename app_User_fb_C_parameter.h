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
#define SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H

//------------------------------------------------------------------------------------//
// C++ compatibility  // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

#include "ssm_std_define.h"
#include "app_User_fb_C_control_type.h"

#define APP_FB_FF_TABLE_SIZE       (12)

typedef struct
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t integral_limit;

    /*
     * Legacy field kept for source/binary compatibility.
     * PID execution does not use this value as an actuator limit.
     */
    int32_t output_limit;

    /* Back-calculation gain, Q15. */
    int32_t kaw;

} APP_FB_PID_PARAMETER_T;

typedef struct
{
    APP_FB_TEMP temp[APP_FB_FF_TABLE_SIZE];
    APP_FB_PWM pwm[APP_FB_FF_TABLE_SIZE];
    int32_t learn_step;
    int32_t learn_limit;

} APP_FB_FF_PARAMETER_T;

#define APP_FB_GAIN_ZONE_NUM       (3)

typedef struct
{
    APP_FB_TEMP low;
    APP_FB_TEMP high;
    APP_FB_PID_PARAMETER_T pid;

} APP_FB_GAIN_ZONE_T;

typedef struct
{
    int32_t enable_error;

} APP_FB_INTEGRAL_SEPARATION_PARAMETER_T;

typedef struct
{
    int32_t alpha;

} APP_FB_D_FILTER_PARAMETER_T;

typedef struct
{
    int32_t rise_limit;
    int32_t fall_limit;

} APP_FB_RATE_LIMIT_PARAMETER_T;

typedef struct
{
    int32_t kaw;

} APP_FB_ANTI_WINDUP_PARAMETER_T;

typedef struct
{
    int32_t error_threshold;
    int32_t gain;

} APP_FB_ADAPTIVE_PARAMETER_T;

/* Q15 defaults: 32768 = 1.0 */
#define APP_FB_PID_KP_DEFAULT          32768
#define APP_FB_PID_KI_DEFAULT          600
#define APP_FB_PID_KD_DEFAULT          65536
#define APP_FB_PID_INTEGRAL_LIMIT      3000

/* Legacy compatibility only; no longer used to clamp PID output. */
#define APP_FB_PID_OUTPUT_LIMIT        800

/*
 * Anti-windup back-calculation.
 *
 * The PID integral state is stored in error-count samples. Therefore the
 * back-calculation correction must be converted from PWM units to integral
 * state units using Kaw / Ki:
 *
 *     I(k+1) += (Kaw / Ki) * (u_sat - u_raw)
 *
 * Kaw = 0.05 is intentionally more conservative than the previous 0.25
 * default. With Ki = 600/32768 this gives Kaw/Ki ~= 2.73 integral-counts
 * per PWM saturation count, reducing aggressive integral unwinding while
 * preserving standard back-calculation semantics.
 */
#define APP_FB_PID_KAW_DEFAULT         1638

/* Maximum anti-windup integral correction per controller cycle (Ts=20ms). */
#define APP_FB_PID_AW_MAX_CORRECTION   300

/* Integral Separation: 2℃ */
#define APP_FB_I_ENABLE_ERROR          20

/* Derivative LPF: alpha = 0.875 */
#define APP_FB_D_FILTER_ALPHA          28672

/* PWM Rate Limit: per 20ms */
#define APP_FB_PWM_RISE_LIMIT          30
#define APP_FB_PWM_FALL_LIMIT          50

/* Anti Windup: Kaw = 0.05 */
#define APP_FB_KAW                     APP_FB_PID_KAW_DEFAULT

/* Adaptive */
#define APP_FB_ADAPTIVE_ERROR          10
#define APP_FB_ADAPTIVE_GAIN           512

#ifdef __cplusplus
}
#endif

#endif  // SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_
