/***************************************************************
Description :
	This is a user USER_DERIVATIVE_FILTER program application header.

	author : CH.

	modify :
			0.setup first version , ch@2026/05/23
			1.move PV history ownership into derivative filter

	version: V0002
***************************************************************/
#ifndef SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_
#define SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_

#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
====================================================
 Derivative Low Pass Filter

 Input : PV, unit 0.1°C
 Output: Filtered dPV, unit 0.1°C / sample

 The derivative filter owns the previous-PV state so the
 controller does not need to access PID internal state.
 The first execution after init/reset returns 0 dPV to
 prevent a startup derivative kick.
====================================================
*/
typedef struct
{
    /* Filter coefficient, Q15, 32768 = 1.0 */
    int32_t alpha;

    /* Previous filtered derivative output */
    int32_t output;

    /* Previous PV used to calculate dPV */
    int32_t pv_previous;

    /* Previous PV is valid after the first run */
    APP_FB_BOOL initialized;

} APP_FB_D_FILTER_T;

void app_fb_d_filter_init
(
    APP_FB_D_FILTER_T *fb,
    int32_t alpha
);

void app_fb_d_filter_reset
(
    APP_FB_D_FILTER_T *fb
);

/*
====================================================
 Run

 Input : current PV
 Output: filtered dPV

 dPV(k) = PV(k) - PV(k-1)
 y(k)   = alpha*y(k-1) + (1-alpha)*dPV(k)

 First run after init/reset:
 dPV = 0, y = 0
====================================================
*/
MY_API int32_t app_fb_d_filter_run
(
    APP_FB_D_FILTER_T *fb,
    int32_t pv
);

#ifdef __cplusplus
}
#endif

#endif  // SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_
