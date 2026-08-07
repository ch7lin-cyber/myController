/***************************************************************
Description : 
	This is a user USER_DERIVATIVE_FILTER program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_
#define SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_


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
 Derivative Low Pass Filter
 Input: Raw derivative
 Output: Filtered derivative
 Unit: 0.1℃ / sample
====================================================
*/
typedef struct
{
    /*
     * Filter coefficient
     * Q15
     * 32768 = 1.0
     */
    int32_t alpha;

    /*
     * Previous output
     */
    int32_t output;

}APP_FB_D_FILTER_T;


/*
====================================================
 Initialize

====================================================
*/
MY_API void app_fb_d_filter_init
(
    APP_FB_D_FILTER_T *fb,
    int32_t alpha
);

/*
====================================================
 Reset

====================================================
*/
MY_API void app_fb_d_filter_reset
(
    APP_FB_D_FILTER_T *fb
);


/*
====================================================
 Run
 Input: raw derivative
 Output: filtered derivative
====================================================
*/
MY_API int32_t app_fb_d_filter_run
(
    APP_FB_D_FILTER_T *fb, 
    int32_t input
);



//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_




