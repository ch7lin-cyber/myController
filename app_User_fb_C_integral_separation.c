
/***************************************************************
Description : 
	This is a user APP_USER_INTEGRAL_SEPARATION program application.


------------------------------------------------------------------------------------------------------------------------------------------
Change notice:

Date-> 2026/05/13
[ADD] 1. The first version sets up. 

[MODIFY] 1. The first version sets up. 

[DELETE] 1. The first version sets up. 

**************************************************************************************/


#include "app_User_fb_C_integral_separation.h"


//------------------------------------------------------------------------------------//
// C++ compatibility // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

/*
====================================================
 Hysteresis
 Unit: 0.1℃
 Example:
 Threshold = 20
 Hysteresis = 5
 Enable:  Error <= 20
 Disable: Error >= 25
====================================================
*/

#define APP_FB_I_HYSTERESIS     (5)

/*
====================================================
 Initialize
====================================================
*/
MY_API void app_fb_integral_separation_init
(
    APP_FB_INTEGRAL_SEPARATION_T *fb,
    int32_t threshold
)
{

    if(fb == 0)
        return;

    if(threshold < 0)
        threshold = 0;
	
    fb->error_threshold = threshold;
    fb->enable = APP_FB_FALSE;
}


/*
====================================================
 Run
 Input: Error
 Unit:  0.1℃
 Output:  Integral enable
====================================================
*/
MY_API APP_FB_BOOL app_fb_integral_separation_run
(
    APP_FB_INTEGRAL_SEPARATION_T *fb, 
    int32_t error 
)
{

    int32_t abs_error;

    if(fb == 0)
        return APP_FB_FALSE;

    abs_error = APP_FB_ABS(error);

    /*
     ================================================
     Enable condition
     ================================================
    */
    if(fb->enable == APP_FB_FALSE)
    {

        if(abs_error <= fb->error_threshold)
        {
            fb->enable = APP_FB_TRUE;
        }
    }

    /*
     ================================================
     Disable condition
     With hysteresis
     ================================================
    */
    else
    {
        if(abs_error >= (fb->error_threshold  +  APP_FB_I_HYSTERESIS )   )
        {
            fb->enable = APP_FB_FALSE;
        }
    }
    return fb->enable;
}




//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//


