
/***************************************************************
Description : 
	This is a user User fb C program application header.


------------------------------------------------------------------------------------------------------------------------------------------
Change notice:

Date-> 2026/05/13
[ADD] 1. The first version sets up. 

[MODIFY] 1. The first version sets up. 

[DELETE] 1. The first version sets up. 

**************************************************************************************/
#ifndef SSM_STD_FB_APP_C_CONTROL_CODE_H_
#define SSM_STD_FB_APP_C_CONTROL_CODE_H_


#include "app_User_fb_C_control_type.h"
#include "app_User_fb_C_parameter.h"



//------------------------------------------------------------------------------------//
// C++ compatibility // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//


/*
====================================================
 PID Internal State
====================================================
*/
typedef struct
{

    /*
     * Integral accumulator
     */
    int32_t integral;

    /*
     * Previous error
     */
    int32_t error_previous;

    /*
     * Previous PV
     */
    int32_t pv_previous;
	
    /*
     * PID output
     */

    int32_t output;

}APP_FB_PID_STATE_T;


/*
====================================================
 PID Function Block
====================================================
*/
typedef struct
{

    /*
     * Parameter
     */
    APP_FB_PID_PARAMETER_T param;

    /*
     * State
     */
    APP_FB_PID_STATE_T state;

    /*
     * Enable
     */
    APP_FB_BOOL enable;

    /*
     * Integral enable
     */
    APP_FB_BOOL integral_enable;

}APP_FB_PID_T;

/*
====================================================
 Initialize
====================================================
*/
void
app_fb_pid_init
(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param
);


/*
====================================================
 Reset
====================================================
*/
void app_fb_pid_reset
(
    APP_FB_PID_T *fb
);


/*
====================================================
 PID Execute
 Input:
 SV: 0.1℃
 PV: 0.1℃
 Output: PID correction PWM -150 ~ +150
====================================================
*/
int32_t app_fb_pid_run
(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv
);

/*
====================================================
 External Integral Correction
 Anti Windup use
====================================================
*/
void app_fb_pid_integral_add
(
    APP_FB_PID_T *fb,
    int32_t value
);


//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_C_CONTROL_CODE_H_

