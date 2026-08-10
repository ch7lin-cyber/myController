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

typedef struct
{
    uint32_t time_constant_ms;
} APP_FB_D_FILTER_PARAMETER_T;

typedef struct
{
    int32_t rise_rate_per_sec;
    int32_t fall_rate_per_sec;
} APP_FB_RATE_LIMIT_PARAMETER_T;

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

/*
 * Timing configuration is owned by the outer application layer.
 * Public unit is milliseconds because application schedulers/HMI parameters
 * commonly use ms. The controller converts and stores this internally in us.
 */
typedef struct
{
    uint32_t sample_time_ms;
} APP_FB_TIMING_PARAMETER_T;

#define APP_FB_PID_KP_DEFAULT 32768
#define APP_FB_PID_KI_DEFAULT 600
#define APP_FB_PID_KD_DEFAULT 65536
#define APP_FB_PID_INTEGRAL_LIMIT 3000
#define APP_FB_PID_OUTPUT_LIMIT 800
#define APP_FB_PID_KAW_DEFAULT 1638
#define APP_FB_PID_AW_MAX_CORRECTION 300

/* PID gains remain expressed using the legacy 20 ms reference convention.
 * B4-T2 converts them once during initialization to runtime discrete gains. */
#define APP_FB_PID_REFERENCE_SAMPLE_TIME_MS  (20U)

/* Integral approach-zone tuning (temperature unit = 0.1 degC). */
#define APP_FB_I_ENABLE_ERROR        15
#define APP_FB_I_HYSTERESIS           5
#define APP_FB_I_SV_CHANGE_THRESHOLD  5

/*
 * Fast-heating boost for heater-only plants.
 * It applies only when PV is below SV (positive error).
 * Temperature error unit = 0.1 degC, PWM unit = 0.1%.
 *
 * error <= 20.0C : normal FF + PID only
 * error  = 20.0C : boost floor starts at 50% PWM
 * error  = 80.0C : boost floor reaches 100% PWM
 * between the two points the floor is linearly interpolated.
 */
#define APP_FB_FAST_HEAT_ENABLE             (1)
#define APP_FB_FAST_HEAT_START_ERROR        (200)
#define APP_FB_FAST_HEAT_FULL_ERROR         (800)
#define APP_FB_FAST_HEAT_START_PWM          (500)
#define APP_FB_FAST_HEAT_FULL_PWM           (1000)

/*
 * Derivative LPF physical time constant.
 * Legacy alpha was 28672 (0.875) at Ts=20 ms.
 * tau=140 ms gives alpha=tau/(tau+Ts)=140/(140+20)=0.875,
 * therefore preserving the original 20 ms behavior exactly.
 */
#define APP_FB_D_FILTER_TIME_CONSTANT_MS  (140U)

/*
 * Output slew-rate limits are physical PWM counts per second.
 * Legacy behavior at Ts=20 ms was +30 / -50 counts per cycle:
 *   rise = 30 / 0.020 = 1500 counts/s
 *   fall = 50 / 0.020 = 2500 counts/s
 */
#define APP_FB_PWM_RISE_RATE_PER_SEC  (1500)
#define APP_FB_PWM_FALL_RATE_PER_SEC  (2500)

#define APP_FB_KAW APP_FB_PID_KAW_DEFAULT
#define APP_FB_ADAPTIVE_ERROR 10
#define APP_FB_ADAPTIVE_GAIN 512
#define APP_FB_ADAPTIVE_SV_CHANGE 5
#define APP_FB_ADAPTIVE_PID_DEADBAND 10
#define APP_FB_ADAPTIVE_ERROR_DEADBAND 3

/*
 * Adaptive-learning timing is expressed in physical milliseconds.
 * Legacy Ts=20 ms behavior:
 *   stable_count = 50  -> 1000 ms
 *   freeze_count = 250 -> 5000 ms
 */
#define APP_FB_ADAPTIVE_STABLE_TIME_MS  (1000U)
#define APP_FB_ADAPTIVE_FREEZE_TIME_MS  (5000U)
#define APP_FB_ADAPTIVE_OFFSET_LIMIT 200

/* Runtime adaptive-learning validation limits. */
#define APP_FB_ADAPTIVE_ERROR_MAX            (APP_FB_TEMP_MAX - APP_FB_TEMP_MIN)
#define APP_FB_ADAPTIVE_GAIN_MAX             (APP_FB_Q15_ONE)
#define APP_FB_ADAPTIVE_SV_CHANGE_MIN        (1)
#define APP_FB_ADAPTIVE_SV_CHANGE_MAX        (APP_FB_TEMP_MAX - APP_FB_TEMP_MIN)
#define APP_FB_ADAPTIVE_PID_DEADBAND_MAX     (APP_FB_PWM_MAX)
#define APP_FB_ADAPTIVE_OFFSET_LIMIT_MAX     (APP_FB_PWM_MAX)
#define APP_FB_ADAPTIVE_TIME_MAX_MS          (86400000U) /* 24 h guard */

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
