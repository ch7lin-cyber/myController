/***************************************************************
Description : 
	This is a user USER_FEEDFORWARD_TABLE program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_FEEDFORWARD_TABLE_CODE_H_
#define SSM_STD_FB_APP_USER_FEEDFORWARD_TABLE_CODE_H_


#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"


//------------------------------------------------------------------------------------//
// C++ compatibility  // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

/*
====================================================
 Feedforward Table Data
====================================================
*/

typedef struct
{
    APP_FB_TEMP temperature;
    APP_FB_PWM pwm;
}APP_FB_FF_POINT_T;

/*
====================================================
 Feedforward Table Instance
====================================================
*/


typedef struct
{

    /*
     * Table pointer
     */
    const APP_FB_FF_POINT_T *table;

    /*
     * Table size
     */
    int32_t size;

    /*
     * Output
     */
    APP_FB_PWM output;

}APP_FB_FEEDFORWARD_T;

/*
====================================================
 Initialize
====================================================
*/

MY_API void app_fb_feedforward_init
(
    APP_FB_FEEDFORWARD_T *fb,
    const APP_FB_FF_POINT_T *table,
    int32_t size

);

/*
====================================================
 Run
 Input:  SV
 Unit:  0.1℃
 Output:  PWM  0~1000
====================================================
*/
MY_API APP_FB_PWM app_fb_feedforward_run
(
    APP_FB_FEEDFORWARD_T *fb,
    APP_FB_TEMP sv
);

//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_LOGIC_CODE_H_




