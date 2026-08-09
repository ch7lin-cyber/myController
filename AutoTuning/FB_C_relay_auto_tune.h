#ifndef FB_C_RELAY_AUTO_TUNE_H_
#define FB_C_RELAY_AUTO_TUNE_H_

#include <stdint.h>
#include "../PID_ControllerSrc/FB_C_control_type.h"
#include "../PID_ControllerSrc/FB_C_parameter.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef enum
{
    APP_FB_AUTOTUNE_RULE_PI_CONSERVATIVE = 0,
    APP_FB_AUTOTUNE_RULE_PID_CONSERVATIVE
} APP_FB_AUTOTUNE_RULE_T;

typedef enum
{
    APP_FB_AUTOTUNE_IDLE = 0,
    APP_FB_AUTOTUNE_RUNNING,
    APP_FB_AUTOTUNE_DONE,
    APP_FB_AUTOTUNE_ERROR
} APP_FB_AUTOTUNE_STATUS_T;

typedef enum
{
    APP_FB_AUTOTUNE_ERROR_NONE = 0,
    APP_FB_AUTOTUNE_ERROR_NULL_POINTER,
    APP_FB_AUTOTUNE_ERROR_PARAMETER,
    APP_FB_AUTOTUNE_ERROR_SAFETY_LIMIT,
    APP_FB_AUTOTUNE_ERROR_TIMEOUT,
    APP_FB_AUTOTUNE_ERROR_NO_OSCILLATION
} APP_FB_AUTOTUNE_ERROR_T;

typedef struct
{
    APP_FB_TEMP sv;
    APP_FB_PWM bias_pwm;
    APP_FB_PWM relay_amplitude;
    APP_FB_TEMP hysteresis;

    uint32_t sample_time_ms;
    uint32_t max_test_time_ms;
    uint16_t min_cycles;

    APP_FB_PWM pwm_min;
    APP_FB_PWM pwm_max;
    APP_FB_TEMP pv_min;
    APP_FB_TEMP pv_max;

    APP_FB_AUTOTUNE_RULE_T rule;
} APP_FB_RELAY_AUTOTUNE_CONFIG_T;

typedef struct
{
    double ku;
    double pu_s;
    double pv_amplitude;
    APP_FB_PWM relay_amplitude;
    APP_FB_PID_PARAMETER_T pid;
} APP_FB_RELAY_AUTOTUNE_RESULT_T;

typedef struct
{
    APP_FB_RELAY_AUTOTUNE_CONFIG_T cfg;
    APP_FB_RELAY_AUTOTUNE_RESULT_T result;

    APP_FB_AUTOTUNE_STATUS_T status;
    APP_FB_AUTOTUNE_ERROR_T error;

    APP_FB_PWM output_pwm;
    APP_FB_BOOL relay_high;

    uint32_t sample_count;
    uint32_t last_high_to_low_sample;
    uint64_t period_sample_sum;
    uint16_t period_count;

    APP_FB_TEMP segment_max;
    APP_FB_TEMP segment_min;
    int64_t peak_sum;
    int64_t trough_sum;
    uint16_t peak_count;
    uint16_t trough_count;

    APP_FB_BOOL first_high_to_low_seen;
} APP_FB_RELAY_AUTOTUNE_T;

void app_fb_relay_autotune_init(
    APP_FB_RELAY_AUTOTUNE_T *fb,
    const APP_FB_RELAY_AUTOTUNE_CONFIG_T *cfg);

void app_fb_relay_autotune_start(
    APP_FB_RELAY_AUTOTUNE_T *fb,
    APP_FB_TEMP initial_pv);

APP_FB_PWM app_fb_relay_autotune_run(
    APP_FB_RELAY_AUTOTUNE_T *fb,
    APP_FB_TEMP pv);

void app_fb_relay_autotune_abort(APP_FB_RELAY_AUTOTUNE_T *fb);

APP_FB_AUTOTUNE_STATUS_T app_fb_relay_autotune_get_status(
    const APP_FB_RELAY_AUTOTUNE_T *fb);

APP_FB_AUTOTUNE_ERROR_T app_fb_relay_autotune_get_error(
    const APP_FB_RELAY_AUTOTUNE_T *fb);

APP_FB_BOOL app_fb_relay_autotune_get_result(
    const APP_FB_RELAY_AUTOTUNE_T *fb,
    APP_FB_RELAY_AUTOTUNE_RESULT_T *result);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* FB_C_RELAY_AUTO_TUNE_H_ */
