
/***************************************************************
Description : 
	This is a user USER_DERIVATIVE_FILTER program application.


------------------------------------------------------------------------------------------------------------------------------------------
Change notice:

Date-> 2026/05/13
[ADD] 1. The first version sets up. 

[MODIFY] 1. The first version sets up. 

[DELETE] 1. The first version sets up. 

**************************************************************************************/


#include "app_User_fb_C_derivative_filter.h"


//------------------------------------------------------------------------------------//
// C++ compatibility // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//




/*
====================================================
 Initialize
====================================================
*/


MY_API void
app_fb_d_filter_init
(
    APP_FB_D_FILTER_T *fb,
    int32_t alpha
)
{

    if(fb == 0)
        return;

    /*
     * Q15
     * 0~32767
     */
    if(alpha < 0)
        alpha = 0;
    if(alpha > 32767)
        alpha = 32767;
    fb->alpha = alpha;
    fb->output = 0;
}


/*
====================================================
 Reset
====================================================
*/

MY_API void app_fb_d_filter_reset
(
    APP_FB_D_FILTER_T *fb
)
{

    if(fb == 0)
        return;
    fb->output = 0;
}

/*
====================================================
 Run
 y(k) =
 alpha*y(k-1) +
 (1-alpha)*x(k)
====================================================
*/
MY_API int32_t app_fb_d_filter_run
(
    APP_FB_D_FILTER_T *fb,
    int32_t input)
{
    int64_t result;

    if(fb == 0)
        return 0;
    /*
     * alpha * previous output
     */

    result =   (int64_t)fb->alpha  *  fb->output;
    result >>= 15;

    /*
     * (1-alpha)*input
     */
    result +=  ( (int64_t)(APP_FB_Q15_ONE - fb->alpha)  *   input  ) >> 15;
    fb->output = (int32_t)result;
    return fb->output;
}






//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//


