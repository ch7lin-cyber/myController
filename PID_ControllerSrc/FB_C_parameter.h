#ifndef SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_
#define SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_

#include <stdint.h>
#include "ssm_std_define.h"
#include "FB_C_control_type.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

#define APP_FB_FF_TABLE_SIZE (12)

typedef struct { int32_t kp; int32_t ki; int32_t kd; int32_t integral_limit; int32_t output_limit; int32_t kaw; } APP_FB_PID_PARAMETER_T;
typedef struct { APP_FB_TEMP temp[APP_FB_FF_TABLE_SIZE]; APP_FB_PWM pwm[APP_FB_FF_TABLE_SIZE]; int32_t learn_step; int32_t learn_limit; } APP_FB_FF_PARAMETER_T;
#define APP_FB_GAIN_ZONE_NUM (3)
typedef struct { APP_FB_TEMP low; APP_FB_TEMP high; APP_FB_PID_PARAMETER_T pid; } APP_FB_GAIN_ZONE_T;
typedef struct { int32_t enable_error; } APP_FB_INTEGRAL_SEPARATION_PARAMETER_T;
typedef struct { int32_t alpha; } APP_FB_D_FILTER_PARAMETER_T;
typedef struct { int32_t rise_limit; int32_t fall_limit; } APP_FB_RATE_LIMIT_PARAMETER_T;
typedef struct { int32_t kaw; } APP_FB_ANTI_WINDUP_PARAMETER_T;
typedef struct { int32_t error_threshold; int32_t gain; int32_t sv_change_threshold; int32_t pid_deadband; uint16_t stable_count; uint16_t freeze_count; int32_t offset_limit; } APP_FB_ADAPTIVE_PARAMETER_T;

#define APP_FB_PID_KP_DEFAULT 32768
#define APP_FB_PID_KI_DEFAULT 600
#define APP_FB_PID_KD_DEFAULT 65536
#define APP_FB_PID_INTEGRAL_LIMIT 3000
#define APP_FB_PID_OUTPUT_LIMIT 800
#define APP_FB_PID_KAW_DEFAULT 1638
#define APP_FB_PID_AW_MAX_CORRECTION 300

/* Integral approach-zone tuning (temperature unit = 0.1 degC).
 * From large error, integral remains disabled until |error| <= 1.5 degC.
 * Once the controller has entered this zone at a fixed SV, integral becomes
 * disturbance-armed and remains available for later load rejection.
 * A meaningful SV change clears the armed state and the old integral state.
 */
#define APP_FB_I_ENABLE_ERROR        15
#define APP_FB_I_HYSTERESIS           5
#define APP_FB_I_SV_CHANGE_THRESHOLD  5

#define APP_FB_D_FILTER_ALPHA 28672
#define APP_FB_PWM_RISE_LIMIT 30
#define APP_FB_PWM_FALL_LIMIT 50
#define APP_FB_KAW APP_FB_PID_KAW_DEFAULT
#define APP_FB_ADAPTIVE_ERROR 10
#define APP_FB_ADAPTIVE_GAIN 512
#define APP_FB_ADAPTIVE_SV_CHANGE 5
#define APP_FB_ADAPTIVE_PID_DEADBAND 10
#define APP_FB_ADAPTIVE_ERROR_DEADBAND 3
#define APP_FB_ADAPTIVE_STABLE_COUNT APP_FB_ADAPTIVE_PERIOD
#define APP_FB_ADAPTIVE_FREEZE_COUNT 250
#define APP_FB_ADAPTIVE_OFFSET_LIMIT 200

/* Runtime adaptive-learning validation limits. */
#define APP_FB_ADAPTIVE_ERROR_MAX            (APP_FB_TEMP_MAX - APP_FB_TEMP_MIN)
#define APP_FB_ADAPTIVE_GAIN_MAX             (APP_FB_Q15_ONE)
#define APP_FB_ADAPTIVE_SV_CHANGE_MIN        (1)
#define APP_FB_ADAPTIVE_SV_CHANGE_MAX        (APP_FB_TEMP_MAX - APP_FB_TEMP_MIN)
#define APP_FB_ADAPTIVE_PID_DEADBAND_MAX     (APP_FB_PWM_MAX)
#define APP_FB_ADAPTIVE_OFFSET_LIMIT_MAX     (APP_FB_PWM_MAX)

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
