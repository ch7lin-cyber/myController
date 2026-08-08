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
[MODIFY] 5. Keep PID state declaration synchronized with anti-windup implementation.
[ADD] 6. Add bumpless PID preload for MANUAL -> AUTO transfer.
***************************************************************/
#ifndef SSM_STD_FB_APP_C_CONTROL_CODE_H_
#define SSM_STD_FB_APP_C_CONTROL_CODE_H_

#include <stdint.h>
#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t integral;
    int32_t error_previous;
    int32_t output;
    int64_t aw_remainder;
} APP_FB_PID_STATE_T;

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

int32_t app_fb_pid_run
(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv,
    int32_t d_filtered
);

/*
 * Bumpless MANUAL -> AUTO preload.
 * desired_pid_output is the PID correction required to reproduce the
 * previous manual actuator command after FF and FF learning offset.
 * The function compensates for the integral increment that pid_run()
 * performs in the same controller cycle.
 */
void app_fb_pid_bumpless_preload
(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv,
    int32_t d_filtered,
    int32_t desired_pid_output
);

void app_fb_pid_anti_windup
(
    APP_FB_PID_T *fb,
    int32_t unsaturated_output,
    int32_t actual_output
);

void app_fb_pid_integral_add
(
    APP_FB_PID_T *fb,
    int32_t value
);

#ifdef __cplusplus
}
#endif

#endif  // SSM_STD_FB_APP_C_CONTROL_CODE_H_
