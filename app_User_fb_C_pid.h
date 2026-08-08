/***************************************************************
Description :
	This is a user User fb C program application header.

Change notice:
Date-> 2026/05/13
[ADD] 1. The first version sets up.
[MODIFY] 1. Remove previous PV ownership from PID; derivative state belongs to D filter.
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
