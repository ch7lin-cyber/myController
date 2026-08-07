/***************************************************************
Description : 
	This is a user APP_USER_RATE_LIMIT program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_
#define SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_

#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"


//------------------------------------------------------------------------------------//
// C++ compatibility  // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//
// extern your program for others use.


/*
====================================================
 PWM Output Rate Limiter FB

 Unit: PWM count
 Range: 0~1000
 Example:
 Rise limit = 30
 600
 630
 660
 690
====================================================
*/

typedef struct
{

    /*
     * Rising limit
     * PWM / cycle
     */
    int32_t rise_limit;

    /*
     * Falling limit
     */
    int32_t fall_limit;

    /*
     * Previous output
     */
    APP_FB_PWM previous;

    /*
     * Current output
     */
    APP_FB_PWM output;

}APP_FB_RATE_LIMIT_T;



/*
====================================================
 Initialize
====================================================
*/
MY_API void app_fb_rate_limit_init
(
    APP_FB_RATE_LIMIT_T *fb,
    int32_t rise_limit,
    int32_t fall_limit
);


/*
====================================================
 Reset
====================================================
*/
MY_API void app_fb_rate_limit_reset
(
    APP_FB_RATE_LIMIT_T *fb,
    int32_t output
);


/*
====================================================
 Run
 Input: Target PWM
 Output: Limited PWM
====================================================
*/
MY_API APP_FB_PWM app_fb_rate_limit_run
(
    APP_FB_RATE_LIMIT_T *fb,
    APP_FB_PWM input
);




//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_




