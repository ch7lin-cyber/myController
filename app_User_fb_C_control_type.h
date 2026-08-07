/***************************************************************
Description : 
	This is a user logic program application header.

	author : CH.
	
	modify :
			0.setup first version , ch@2026/05/23
			1.change ..........

	version: V0001
	

***************************************************************/
#ifndef SSM_STD_FB_APP_USER_C_TEST_CODE_H_
#define SSM_STD_FB_APP_USER_C_TEST_CODE_H_

#include "ssm_std_define.h"


//------------------------------------------------------------------------------------//
// C++ compatibility  // DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//
// extern your program for others use.


/*
====================================================
 IEC61131-3 style basic type
====================================================
*/

typedef int32_t APP_FB_INT32;
typedef int64_t APP_FB_INT64;

/*
====================================================
 Temperature
 Unit:
 0.1 degC
 Example:
 175.0 degC
 stored as:
 1750
====================================================
*/

typedef int32_t APP_FB_TEMP;

/*
====================================================
 PWM
 Range:
 0 ~ 1000
 1000 = 100.0 %
====================================================
*/
typedef int32_t APP_FB_PWM;
/*
====================================================
 Percentage
 0~1000
====================================================
*/
typedef int32_t APP_FB_PERCENT;
/*
====================================================
 Boolean
====================================================
*/
typedef enum
{
    APP_FB_FALSE = 0,
    APP_FB_TRUE  = 1
}APP_FB_BOOL;

/*
====================================================
 Function Block State
====================================================
*/
typedef enum
{
    APP_FB_STATE_INIT = 0,
    APP_FB_STATE_IDLE,
    APP_FB_STATE_RUN,
    APP_FB_STATE_HOLD,
    APP_FB_STATE_FAULT
}APP_FB_STATE;

/*
====================================================
 Control Mode
 IEC61131 style
====================================================
*/
typedef enum
{
    APP_FB_MODE_MANUAL = 0,
    APP_FB_MODE_AUTO
}APP_FB_CONTROL_MODE;
/*
====================================================
 Error Code
====================================================
*/
typedef enum
{
    APP_FB_OK = 0,
    APP_FB_ERROR_NULL_POINTER,
    APP_FB_ERROR_SENSOR,
    APP_FB_ERROR_PARAMETER,
    APP_FB_ERROR_OUTPUT
}APP_FB_ERROR;


/*
====================================================
 Fixed Point
 Q15
 1.0 = 32768

====================================================
*/
#define APP_FB_Q15_ONE          (32768)

/*
====================================================
 Controller Timing
====================================================
*/

/*
 Control loop:
 50Hz
 Ts = 20ms
*/

#define APP_FB_CONTROL_PERIOD_MS     (20)

/*
 Adaptive update:
 1 second
*/

#define APP_FB_ADAPTIVE_PERIOD       (50)

/*
====================================================
 Temperature Range

====================================================
*/
#define APP_FB_TEMP_MIN              (0)
#define APP_FB_TEMP_MAX              (3000)
/*
====================================================
 PWM Range

====================================================
*/
#define APP_FB_PWM_MIN               (0)
#define APP_FB_PWM_MAX               (1000)

/*
====================================================
 Utility Macro
====================================================
*/
#define APP_FB_ABS(x)       \
        (((x)>=0)?(x):(-(x)))

#define APP_FB_LIMIT(x,min,max)       \
        (((x)<(min))?(min):(((x)>(max))?(max):(x)))



//------------------------------------------------------------------------------------//
// C++ compatibility
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//
#endif  // SSM_STD_FB_APP_USER_C_TEST_CODE_H_




