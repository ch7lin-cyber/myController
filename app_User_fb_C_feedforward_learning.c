
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


#include "app_User_fb_C_feedforward_learning.h"


//------------------------------------------------------------------------------------//
// C++ compatibility // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//


/*
====================================================
 Learning Offset Limit
 PWM count
 防止一次學太多
====================================================
*/

#define APP_FB_FF_LEARN_MAX_OFFSET     (200)
#define APP_FB_FF_LEARN_MIN_OFFSET     (-200)

/*
====================================================
 Local Limit
====================================================
*/
static int32_t app_fb_ff_learning_limit
(
    int32_t value
)
{

    if(value > APP_FB_FF_LEARN_MAX_OFFSET)
        return APP_FB_FF_LEARN_MAX_OFFSET;

    if(value < APP_FB_FF_LEARN_MIN_OFFSET)
        return APP_FB_FF_LEARN_MIN_OFFSET;

    return value;
}


/*
====================================================
 Initialize
====================================================
*/
MY_API void app_fb_ff_learning_init
(
    APP_FB_FF_LEARNING_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *param
)
{
    if(fb == 0)
	{
        return;
	}
	
    fb->error_threshold =  APP_FB_ADAPTIVE_ERROR;

    fb->gain = APP_FB_ADAPTIVE_GAIN;

 



    /*
     * 50Hz
     * 50 cycles = 1 sec
     */

    
    fb->offset = 0;
	
	fb->pid_sum = 0;
	fb->pid_count = 0;

	fb->stable_counter = 0;
	fb->freeze_counter = 0;

	if (fb->previous_sv == 0)
	{
		fb->previous_sv = 0;
	}
 


}


/*
====================================================
 Reset
====================================================
*/
MY_API void app_fb_ff_learning_reset
(
    APP_FB_FF_LEARNING_T *fb
)
{

    if(fb == 0)
	{
        return;
	}
	

	fb->offset = 0;

	fb->pid_sum = 0;

	fb->pid_count = 0;

	fb->stable_counter = 0;

	fb->freeze_counter = 0;

	fb->previous_sv = 0;
}


/*
====================================================
 Run
 Input: PID output
 Example: PID = +30
 Means: FF insufficient
====================================================
*/

MY_API int32_t app_fb_ff_learning_run
(
    APP_FB_FF_LEARNING_T *fb,
    int32_t sv,
    int32_t pv,
    int32_t pid_output
)
{

    int64_t learn;
	
	if(fb == 0)
        return 0;
	
	
	/*--------------------------------------------------
	 * Detect SV change
	 *-------------------------------------------------*/
	if (abs(sv - fb->previous_sv) >= 5)
	{
		fb->previous_sv = (int16_t)sv;

		fb->freeze_counter = APP_FB_FF_FREEZE_COUNT;

		fb->stable_counter = 0;
		fb->pid_sum = 0;
		fb->pid_count = 0;
	}

	

	/*--------------------------------------------------
	 * Freeze learning after SV changed
	 *-------------------------------------------------*/
	if (fb->freeze_counter > 0)
	{
		fb->freeze_counter--;

		return fb->offset;
	}
	
	

	/******************
		Error 檢查
	*******************/


	int32_t error;

	error = sv - pv;

	if(error > fb->error_threshold)
	{
		fb->stable_counter = 0;
		fb->pid_sum = 0;
		fb->pid_count = 0;

		return fb->offset;
	}

	if(error < -fb->error_threshold)
	{
		fb->stable_counter = 0;
		fb->pid_sum = 0;
		fb->pid_count = 0;

		return fb->offset;
	}

	
	/******************
		PID 檢查
	*******************/
	if(pid_output > APP_FB_FF_PID_DEADBAND)
	{
		fb->stable_counter = 0;
		fb->pid_sum = 0;
		fb->pid_count = 0;

		return fb->offset;
	}

	if(pid_output < -APP_FB_FF_PID_DEADBAND)
	{
		fb->stable_counter = 0;
		fb->pid_sum = 0;
		fb->pid_count = 0;

		return fb->offset;
	}

	/************************
		開始累積
	************************/
	fb->stable_counter++;

	fb->pid_sum += pid_output;

	fb->pid_count++;
	
	/***************************
		250 次判斷
	****************************/

	if(fb->stable_counter < APP_FB_FF_STABLE_COUNT)
    {
        return fb->offset;
	}
	

    /*
     * Update every 1 second
     */

	
	/*
	 * Learning:
	 *
	 * offset += PID * gain
	 *
	 */
	int32_t avg_pid;
	
	if (fb->pid_count == 0)
	{
		return fb->offset;
	}

	avg_pid = fb->pid_sum / fb->pid_count;

	learn = (int64_t)avg_pid * fb->gain;
	

	learn >>=15;

	fb->offset += (int32_t)learn;
	
	fb->offset = app_fb_ff_learning_limit(fb->offset);
	
	fb->stable_counter = 0;

	fb->pid_sum = 0;

	fb->pid_count = 0;
	
	return fb->offset;






	
}

/*
====================================================
 Get Offset
====================================================
*/
MY_API int32_t app_fb_ff_learning_get_offset
(
    APP_FB_FF_LEARNING_T *fb
)
{
    if(fb == 0)
        return 0;

    return fb->offset;
}



//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//


