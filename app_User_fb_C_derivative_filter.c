/***************************************************************
Description :
	This is a user USER_DERIVATIVE_FILTER program application.

Change notice:
Date-> 2026/05/13
[ADD] 1. The first version sets up.
[MODIFY] 1. Move PV history and dPV calculation into filter.
***************************************************************/

#include "app_User_fb_C_derivative_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_fb_d_filter_init
(
    APP_FB_D_FILTER_T *fb,
    int32_t alpha
)
{
    if(fb == 0)
        return;

    if(alpha < 0)
        alpha = 0;
    if(alpha > 32767)
        alpha = 32767;

    fb->alpha = alpha;
    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
}

void app_fb_d_filter_reset
(
    APP_FB_D_FILTER_T *fb
)
{
    if(fb == 0)
        return;

    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
}

/*
====================================================
 Run

 dPV(k) = PV(k) - PV(k-1)
 y(k)   = alpha*y(k-1) + (1-alpha)*dPV(k)

 The filter owns previous PV.  On the first execution after
 init/reset, previous PV is initialized from the current PV
 and the derivative is forced to zero.  This prevents a
 startup D kick caused by an invalid previous PV.
====================================================
*/
MY_API int32_t app_fb_d_filter_run
(
    APP_FB_D_FILTER_T *fb,
    int32_t pv
)
{
    int32_t d_pv;
    int64_t result;

    if(fb == 0)
        return 0;

    if(fb->initialized == APP_FB_FALSE)
    {
        fb->pv_previous = pv;
        fb->output = 0;
        fb->initialized = APP_FB_TRUE;
        return 0;
    }

    d_pv = pv - fb->pv_previous;
    fb->pv_previous = pv;

    result = ((int64_t)fb->alpha * fb->output) >> 15;
    result += ((int64_t)(APP_FB_Q15_ONE - fb->alpha) * d_pv) >> 15;

    fb->output = (int32_t)result;
    return fb->output;
}

#ifdef __cplusplus
}
#endif
