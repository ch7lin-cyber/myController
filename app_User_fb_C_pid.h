/***************************************************************
Description :
	This is a user User fb C program application header.

Change notice:
Date-> 2026/05/13
[ADD] 1. The first version sets up.
[MODIFY] 1. Remove previous PV ownership from PID; derivative state belongs to D filter.
[MODIFY] 2. Add back-calculation anti-windup interface.
[MODIFY] 3. Clarify anti-windup output semantics: hard PWM saturation only; rate limiting is excluded.
[MODIFY] 4. Add persistent anti-windup remainder state for fractional correction.
***************************************************************/
#ifndef SSM_STD_FB_APP_C_CONTROL_CODE_H_
#define SSM_STD_FB_APP_C_CONTROL_CODE_H_

#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
====================================================
 PID Internal State
====================================================
*/
typedef struct
{
    /* Integral accumulator */
    int32_t integral;

    /* Previous error */
    int32_t error_previous;

    /* PID output */
    int32_t output;

    /*
     * Fractional back-calculation remainder.
     * Units: numerator remainder in Kaw*PWM units.
     * This preserves sub-integer integral correction between cycles.
     */
    int64_t aw_remainder;

} APP_FB_PID_STATE_T;

/*
====================================================
 PID Function Block
====================================================
*/
typedef struct
{
    APP_FB_PID_PARAMETER_T param;
    APP_FB_PID_STATE_T state;
    APP_FB_BOOL enable;
    APP_FB_BOOL integral_enable;

} APP_FB_PID_T;

void app_fb_pid_init
(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param
);

void app_fb_pid_reset
(
    APP_FB_PID_T *fb
);

/*
====================================================
 PID Execute
 Input:
 SV: 0.1°C
 PV: 0.1°C
 D input: filtered dPV, 0.1°C / sample
 Output: PID correction PWM

 Derivative history is owned by the derivative-filter FB.
====================================================
*/
int32_t app_fb_pid_run
(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv,
    int32_t d_filtered
);

/*
====================================================
 Back Calculation Anti-Windup

 unsaturated_output:
     Combined controller command before hard PWM saturation.

 actual_output:
     PWM command after hard PWM saturation and before output
     rate limiting. The rate-limited actuator output MUST NOT be
     passed to this function.

 The correction is applied to the PID integral accumulator.
 The per-cycle correction limit is configured by
 APP_FB_PID_AW_MAX_CORRECTION in app_User_fb_C_parameter.h.
 Fractional correction is retained in the PID state so integer
 truncation does not accumulate a systematic bias.
====================================================
*/
void app_fb_pid_anti_windup
(
    APP_FB_PID_T *fb,
    int32_t unsaturated_output,
    int32_t actual_output
);

/* External Integral Correction / Anti-Windup use */
void app_fb_pid_integral_add
(
    APP_FB_PID_T *fb,
    int32_t value
);

#ifdef __cplusplus
}
#endif

#endif  // SSM_STD_FB_APP_C_CONTROL_CODE_H_
