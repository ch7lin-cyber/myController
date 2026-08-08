/***************************************************************
Description : 
	This is a user User fb C program application.


------------------------------------------------------------------------------------------------------------------------------------------
Change notice:

Date-> 2026/05/13
[ADD] 1. The first version sets up. 

[MODIFY] 1. The first version sets up. 

[DELETE] 1. The first version sets up. 

**************************************************************************************/

#include "app_User_fb_C_pid.h"

//------------------------------------------------------------------------------------//
// C++ compatibility // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//


/*
====================================================
 Local limit function
====================================================
*/
static int32_t
app_fb_pid_limit
(
    int32_t value,
    int32_t min,
    int32_t max
)
{
    if(value > max)
        return max;

    if(value < min)
        return min;
    return value;
}


/*
====================================================
 Initialize
====================================================
*/
MY_API void app_fb_pid_init
(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param
)
{

    if(fb == 0)
        return;

    if(param != 0)
    {
        fb->param = *param;
    }

    app_fb_pid_reset(fb);
    fb->enable = APP_FB_TRUE;
}


/*
====================================================
 Reset
====================================================
*/
MY_API void app_fb_pid_reset( APP_FB_PID_T *fb )
{
    if(fb == 0)
        return;

    fb->state.integral = 0;
    fb->state.error_previous = 0;
    fb->state.pv_previous = 0;
    fb->state.output = 0;
}


/*
====================================================
 PID Execute
 PID = P + I + D

Integral Separation is controlled by the caller through
fb->integral_enable.  The PID FB must not duplicate the
Integral Separation decision because the controller-level
Integral Separation FB also provides hysteresis/state.
====================================================
*/
MY_API int32_t app_fb_pid_run
(
    APP_FB_PID_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv
)
{

    int32_t error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int32_t output;
    int64_t temp;

    if(fb == 0)
        return 0;

    if(fb->enable == APP_FB_FALSE)
        return 0;

    /*
     * Error
     * Unit:
     * 0.1℃
     */
    error = sv - pv;

    /*
     ================================================
     P Term
     P = Kp * Error
     ================================================
    */
    temp = (int64_t)fb->param.kp * error;
    p_term = (int32_t)(temp >> 15);

    /*
     ================================================
     Integral Separation

     The Integral Separation FB is executed by the
     temperature controller before this PID FB is called.
     Do not calculate the condition again here.
     ================================================
    */
    if(fb->integral_enable == APP_FB_TRUE)
    {
        fb->state.integral += error;

        /*
         * Integral Limit
         */
        fb->state.integral = app_fb_pid_limit
        (
            fb->state.integral,
            -fb->param.integral_limit,
            fb->param.integral_limit
        );
    }

    temp = (int64_t)fb->param.ki * fb->state.integral;
    i_term = (int32_t)(temp >> 15);

    /*
     ================================================
     D Term
     D on Measurement
     D = -Kd * dPV

     NOTE:
     Step 1 intentionally keeps the existing D path
     unchanged.  The controller-level derivative filter
     will be connected to this FB in Step 2.
     ================================================
    */
    temp = (int64_t)fb->param.kd * (pv - fb->state.pv_previous);
    d_term = -(int32_t)(temp >> 15);

    /*
     ================================================
     PID Output
     ================================================
    */
    output = p_term + i_term + d_term;

    /*
     * Output Limit
     */
    output = app_fb_pid_limit
    (
        output,
        -fb->param.output_limit,
        fb->param.output_limit
    );

    /*
     * Save state
     */
    fb->state.error_previous = error;
    fb->state.pv_previous = pv;
    fb->state.output = output;
    return output;

}

/*
====================================================
 External Integral Correction
 Anti Windup
====================================================
*/
MY_API void app_fb_pid_integral_add
(
    APP_FB_PID_T *fb,
    int32_t value
)
{

    if(fb == 0)
        return;

    fb->state.integral += value;

    fb->state.integral = app_fb_pid_limit
    (
        fb->state.integral,
        -fb->param.integral_limit,
        fb->param.integral_limit
    );
}





//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//


