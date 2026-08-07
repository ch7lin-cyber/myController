/***************************************************************
Description : 
	This is a user _APP_USER_FF_LEARNING program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_FF_LEARNING_CODE_H_
#define SSM_STD_FB_APP_USER_FF_LEARNING_CODE_H_

#include <stdlib.h>

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
 Feedforward Learning FB
 Update:
 1 second
 Input: PID correction
 Output: PWM correction
====================================================
*/
typedef struct
{
    /*
     * Learning threshold
     * 0.1℃ error
     */
    int32_t error_threshold;
    /*
     * Learning gain
     * Q15
     */
    int32_t gain;

    /*
     * Counter
     */
    int32_t counter;


    /*
     * Learned offset
     */
    int32_t offset;
	
	/* ---------- New ---------- */

    int32_t pid_sum;

    uint16_t pid_count;

    uint16_t stable_counter;

    uint16_t freeze_counter;

    int16_t previous_sv;
	

}APP_FB_FF_LEARNING_T;


/*
====================================================
 Initialize

====================================================
*/
MY_API void
app_fb_ff_learning_init
(
    APP_FB_FF_LEARNING_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *param
);


/*
====================================================
 Reset
====================================================
*/
MY_API void app_fb_ff_learning_reset
(
    APP_FB_FF_LEARNING_T *fb
);


/*
====================================================
 Run
 Input: PID output
 Unit: PWM count
 Output: FF correction
====================================================
*/
MY_API int32_t app_fb_ff_learning_run
(
	APP_FB_FF_LEARNING_T *fb,
    int32_t sv,
    int32_t pv,
    int32_t pid_output
);

/*
====================================================
 Get offset
====================================================
*/
int32_t app_fb_ff_learning_get_offset
(
    APP_FB_FF_LEARNING_T *fb
);


#define APP_FB_FF_ERROR_DEADBAND        (3)

#define APP_FB_FF_PID_DEADBAND          (10)

#define APP_FB_FF_STABLE_COUNT          (250)

#define APP_FB_FF_FREEZE_COUNT          (250)

#define APP_FB_FF_OFFSET_LIMIT          (100)

#define APP_FB_FF_LEARNING_SHIFT        (6)



//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_FF_LEARNING_CODE_H_




