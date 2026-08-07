/***************************************************************
Description : 
	This is a user APP_USER_INTEGRAL_SEPARATION program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_INTEGRAL_SEPARATION_CODE_H_
#define SSM_STD_FB_APP_USER_INTEGRAL_SEPARATION_CODE_H_


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
 Integral Separation Function Block
====================================================
 Input: Error  Unit: 0.1℃
 Output:  Integral Enable
====================================================
*/
typedef struct
{
    /*
     * Enable threshold
     *
     * Example:
     * 20
     *
     * =2.0℃
     */
    int32_t error_threshold;

    /*
     * Output
     */
    APP_FB_BOOL enable;

}APP_FB_INTEGRAL_SEPARATION_T;



/*
====================================================
 Initialize
====================================================
*/

MY_API void app_fb_integral_separation_init
(
    APP_FB_INTEGRAL_SEPARATION_T *fb,
    int32_t threshold
);


/*
====================================================
 Run
 Input: Error
 Output:  TRUE/FALSE
====================================================
*/

MY_API APP_FB_BOOL app_fb_integral_separation_run
(
    APP_FB_INTEGRAL_SEPARATION_T *fb,
    int32_t error
);


//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_LOGIC_CODE_H_




