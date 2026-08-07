
/***************************************************************
Description : 
	This is a user USER_FEEDFORWARD_TABLE program application.


------------------------------------------------------------------------------------------------------------------------------------------
Change notice:

Date-> 2026/05/13
[ADD] 1. The first version sets up. 

[MODIFY] 1. The first version sets up. 

[DELETE] 1. The first version sets up. 

**************************************************************************************/

#include "app_User_fb_C_feedforward_table.h"

//------------------------------------------------------------------------------------//
// C++ compatibility // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//


/*
====================================================
 Local function
 Linear interpolation
 y = y1 + (x-x1)*(y2-y1)/(x2-x1)
 All int32
====================================================
*/
static int32_t app_fb_ff_interpolation
(
    int32_t x,
    int32_t x1,
    int32_t x2,
    int32_t y1,
    int32_t y2
)
{

    int64_t result;
    /*
     * Avoid divide zero
     */

    if(x2 == x1)
    {
        return y1;
    }

    result =
    (int64_t)(x-x1)    *    (y2-y1);
    result /= (x2-x1);
    result += y1;

    return (int32_t)result;
}

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
)
{
    if(fb == 0)
        return;

    fb->table = table;
    fb->size = size;
    fb->output = 0;
}


/*
====================================================
 Run
 Input:  SV  0.1℃
 Output: PWM  0~1000
====================================================
*/

MY_API APP_FB_PWM  app_fb_feedforward_run
(
    APP_FB_FEEDFORWARD_T *fb,
    APP_FB_TEMP sv
)
{
    int32_t i;
    int32_t pwm;

    if(fb == 0)
        return 0;
    if(fb->table == 0)
        return 0;

    if(fb->size <= 0)
        return 0;

    /*
     * Below minimum
     */

    if(sv <= fb->table[0].temperature)
    {

        fb->output = fb->table[0].pwm;
        return fb->output;
    }

    /*
     * Above maximum
     */

    if(sv >= fb->table[fb->size-1].temperature)
    {
        fb->output =
            fb->table[fb->size-1].pwm;
        return fb->output;
    }


    /*
     * Search table
     */
    for(i=0;i<(fb->size-1);i++)
    {
        if( (sv >= fb->table[i].temperature) && (sv <= fb->table[i+1].temperature)  )
        {
            pwm = app_fb_ff_interpolation
            (
                sv,
                fb->table[i].temperature,
                fb->table[i+1].temperature,
                fb->table[i].pwm,
                fb->table[i+1].pwm
            );

            fb->output =  APP_FB_LIMIT
            (   pwm,
                APP_FB_PWM_MIN,
                APP_FB_PWM_MAX
            );

            return fb->output;

        }
    }

    /*
     * Safety fallback
     */
    fb->output = 0;
    return fb->output;
}



//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//


