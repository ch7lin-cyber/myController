#ifndef SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_
#define SSM_STD_FB_APP_USER_C_PARAMETER_CODE_H_

#include <stdint.h>
#include "ssm_std_define.h"
#include "FB_C_control_type.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FB_FF_TABLE_SIZE (12)

typedef struct { int32_t kp; int32_t ki; int32_t kd; int32_t integral_limit; int32_t output_limit; int32_t kaw; } APP_FB_PID_PARAMETER_T;
typedef struct { APP_FB_TEMP temp[APP_FB_FF_TABLE_SIZE]; APP_FB_PWM pwm[APP_FB_FF_TABLE_SIZE]; int32_t learn_step; int32_t learn_limit; } APP_FB_FF_PARAMETER_T;
#define APP_FB_GAIN_ZONE_NUM (3)
typedef struct { APP_FB_TEMP low; APP_FB_TEMP high; APP_FB_PID_PARAMETER_T pid; } APP_FB_GAIN_ZONE_T;
typedef struct { int32_t enable_error; } APP_FB_INTEGRAL_SEPARATION_PARAMETER_T;
typedef struct { uint32_t time_constant_ms; } APP_FB_D_FILTER_PARAMETER_T;
typedef struct { int32_t rise_rate_per_sec; int32_t fall_rate_per_sec; } APP_FB_RATE_LIMIT_PARAMETER_T;
typedef struct { int32_t kaw; } APP_FB_ANTI_WINDUP_PARAMETER_T;

typedef struct
{
    int32_t error_threshold;
    int32_t gain;
    int32_t sv_change_threshold;
    int32_t pid_deadband;
    uint32_t stable_time_ms;
    uint32_t freeze_time_ms;
    int32_t offset_limit;
} APP_FB_ADAPTIVE_PARAMETER_T;

typedef struct { uint32_t sample_time_ms; } APP_FB_TIMING_PARAMETER_T;

#define APP_FB_PID_KP_DEFAULT 32768
#define APP_FB_PID_KI_DEFAULT 600
#define APP_FB_PID_KD_DEFAULT 65536
#define APP_FB_PID_INTEGRAL_LIMIT 3000
#define APP_FB_PID_OUTPUT_LIMIT 800
#define APP_FB_PID_KAW_DEFAULT 1638
#define APP_FB_PID_AW_MAX_CORRECTION 300
#define APP_FB_PID_REFERENCE_SAMPLE_TIME_MS  (20U)

#define APP_FB_I_ENABLE_ERROR        15
#define APP_FB_I_HYSTERESIS           5
#define APP_FB_I_SV_CHANGE_THRESHOLD  5

#define APP_FB_FAST_HEAT_ENABLE                 (1)
#define APP_FB_FAST_HEAT_ENTER_ERROR             (50)
#define APP_FB_FAST_HEAT_EXIT_ERROR              (30)
#define APP_FB_FAST_HEAT_FULL_ERROR             (300)
#define APP_FB_FAST_HEAT_FULL_PWM              (1000)
#define APP_FB_FAST_HEAT_BLEND_SCALE          (32768)
#define APP_FB_FAST_HEAT_TAIL_ENABLE              (1)
#define APP_FB_FAST_HEAT_TAIL_START_ERROR         (30)
#define APP_FB_FAST_HEAT_TAIL_FULL_ERROR         (100)
#define APP_FB_FAST_HEAT_TAIL_MAX_ADD_PWM         (80)
#define APP_FB_FAST_HEAT_TAIL_CONCAVE_ENABLE       (1)

#define APP_FB_APPROACH_HOLD_ENABLE                (1)
#define APP_FB_APPROACH_HOLD_START_ERROR          (10)
#define APP_FB_APPROACH_HOLD_FULL_ERROR           (30)
#define APP_FB_APPROACH_HOLD_MAX_ADD_PWM          (30)
#define APP_FB_INTEGRAL_FREEZE_ERROR             (300)

#define APP_FB_PREDICTIVE_BRAKE_ENABLE           (1)
#define APP_FB_PREDICTIVE_BRAKE_TIME_DEFAULT_MS (1000U)
extern uint32_t g_app_fb_predictive_brake_time_ms;
#define APP_FB_PREDICTIVE_BRAKE_TIME_MS        (g_app_fb_predictive_brake_time_ms)
#define APP_FB_PREDICTIVE_BRAKE_ENTER_ERROR      (30)
#define APP_FB_PREDICTIVE_BRAKE_FULL_ERROR        (0)
#define APP_FB_PREDICTIVE_BRAKE_MIN_RISE_DELTA    (1)

#define APP_FB_D_FILTER_TIME_CONSTANT_MS  (140U)
#define APP_FB_PWM_RISE_RATE_PER_SEC  (3000)
#define APP_FB_PWM_FALL_RATE_PER_SEC  (7500)

#define APP_FB_KAW APP_FB_PID_KAW_DEFAULT
#define APP_FB_ADAPTIVE_ERROR 10
#define APP_FB_ADAPTIVE_GAIN 512
#define APP_FB_ADAPTIVE_SV_CHANGE 5
#define APP_FB_ADAPTIVE_PID_DEADBAND 10
#define APP_FB_ADAPTIVE_ERROR_DEADBAND 3
#define APP_FB_ADAPTIVE_STABLE_TIME_MS  (1000U)
#define APP_FB_ADAPTIVE_FREEZE_TIME_MS  (5000U)
#define APP_FB_ADAPTIVE_OFFSET_LIMIT 200
#define APP_FB_ADAPTIVE_ERROR_MAX            (APP_FB_TEMP_MAX - APP_FB_TEMP_MIN)
#define APP_FB_ADAPTIVE_GAIN_MAX             (APP_FB_Q15_ONE)
#define APP_FB_ADAPTIVE_SV_CHANGE_MIN        (1)
#define APP_FB_ADAPTIVE_SV_CHANGE_MAX        (APP_FB_TEMP_MAX - APP_FB_TEMP_MIN)
#define APP_FB_ADAPTIVE_PID_DEADBAND_MAX     (APP_FB_PWM_MAX)
#define APP_FB_ADAPTIVE_OFFSET_LIMIT_MAX     (APP_FB_PWM_MAX)
#define APP_FB_ADAPTIVE_TIME_MAX_MS          (86400000U)

/* Branch 5 self-tuning defaults. Temperature unit = 0.1 degC. */
#define APP_FB_SELF_TUNE_SETTLE_BAND                 (2)
#define APP_FB_SELF_TUNE_SETTLE_REQUIRED_MS       (3000U)
#define APP_FB_SELF_TUNE_KI_MIN                    (300)
#define APP_FB_SELF_TUNE_KI_MAX                   (1200)
#define APP_FB_SELF_TUNE_KI_STEP                    (25)
#define APP_FB_SELF_TUNE_STEADY_ERROR_THRESHOLD      (2)
#define APP_FB_SELF_TUNE_OVERSHOOT_THRESHOLD         (5)
#define APP_FB_SELF_TUNE_PREDICTIVE_TIME_MIN_MS    (500U)
#define APP_FB_SELF_TUNE_PREDICTIVE_TIME_MAX_MS   (2000U)
#define APP_FB_SELF_TUNE_PREDICTIVE_TIME_STEP_MS   (100U)
#define APP_FB_SELF_TUNE_MIN_HEATING_STEP            (50)     /* +5.0C */
#define APP_FB_SELF_TUNE_COOLDOWN_MS              (30000U)
#define APP_FB_SELF_TUNE_MAX_COMMITS_PER_SESSION     (20U)

#ifdef __cplusplus
}
#endif
#endif
