/***************************************************************
Description : 
	This is a user logic program application header.
***************************************************************/
#ifndef SSM_STD_FB_APP_USER_C_TEST_CODE_H_
#define SSM_STD_FB_APP_USER_C_TEST_CODE_H_

#include <stdint.h>
#include "ssm_std_define.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef int32_t APP_FB_INT32;
typedef int64_t APP_FB_INT64;
typedef int32_t APP_FB_TEMP;
typedef int32_t APP_FB_PWM;
typedef int32_t APP_FB_PERCENT;

typedef enum { APP_FB_FALSE = 0, APP_FB_TRUE = 1 } APP_FB_BOOL;
typedef enum { APP_FB_STATE_INIT = 0, APP_FB_STATE_IDLE, APP_FB_STATE_RUN, APP_FB_STATE_HOLD, APP_FB_STATE_FAULT } APP_FB_STATE;
typedef enum { APP_FB_MODE_MANUAL = 0, APP_FB_MODE_AUTO } APP_FB_CONTROL_MODE;
typedef enum { APP_FB_OK = 0, APP_FB_ERROR_NULL_POINTER, APP_FB_ERROR_SENSOR, APP_FB_ERROR_PARAMETER, APP_FB_ERROR_OUTPUT } APP_FB_ERROR;

#define APP_FB_Q15_ONE                    (32768)

/*
 * Controller sample-time capability.
 *
 * The application owns the real scheduling period and passes it into the
 * controller during initialization. 20 ms is retained only as the backward-
 * compatible default for legacy init APIs; it is not a fixed runtime
 * controller assumption.
 */
#define APP_FB_SAMPLE_TIME_MIN_MS          (1U)
#define APP_FB_SAMPLE_TIME_MAX_MS          (6000U)
#define APP_FB_SAMPLE_TIME_DEFAULT_MS      (20U)
#define APP_FB_SAMPLE_TIME_US_PER_MS       (1000U)

/*
 * Deprecated compile-compatibility alias for older external code only.
 * Controller-core timing logic must use APP_FB_TIMING_PARAMETER_T instead.
 */
#define APP_FB_CONTROL_PERIOD_MS           APP_FB_SAMPLE_TIME_DEFAULT_MS

#define APP_FB_TEMP_MIN                    (0)
#define APP_FB_TEMP_MAX                    (3000)
#define APP_FB_PWM_MIN                     (0)
#define APP_FB_PWM_MAX                     (1000)
#define APP_FB_ABS(x) (((x)>=0)?(x):(-(x)))
#define APP_FB_LIMIT(x,min,max) (((x)<(min))?(min):(((x)>(max))?(max):(x)))

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
