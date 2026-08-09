#ifndef FB_C_AUTOTUNE_REGRESSION_RUNNER_H_
#define FB_C_AUTOTUNE_REGRESSION_RUNNER_H_

#include <stdint.h>
#include "../PID_ControllerSrc/FB_C_control_type.h"
#include "FB_C_autotune_gain_guard.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef enum
{
    APP_FB_AUTOTUNE_REGRESSION_IDLE = 0,
    APP_FB_AUTOTUNE_REGRESSION_RUNNING,
    APP_FB_AUTOTUNE_REGRESSION_DONE,
    APP_FB_AUTOTUNE_REGRESSION_TIMEOUT,
    APP_FB_AUTOTUNE_REGRESSION_ERROR
} APP_FB_AUTOTUNE_REGRESSION_STATUS_T;

typedef struct
{
    APP_FB_TEMP sv;
    uint32_t sample_time_ms;
    uint32_t max_test_time_ms;

    /* Convergence requires |error| <= convergence_band continuously. */
    APP_FB_TEMP convergence_band;
    uint32_t convergence_hold_ms;

    /* Average steady error over the final steady_window_ms samples. */
    uint32_t steady_window_ms;

    APP_FB_PWM pwm_min;
    APP_FB_PWM pwm_max;
} APP_FB_AUTOTUNE_REGRESSION_CONFIG_T;

typedef struct
{
    APP_FB_AUTOTUNE_REGRESSION_CONFIG_T cfg;
    APP_FB_AUTOTUNE_REGRESSION_STATUS_T status;

    uint32_t sample_count;
    uint32_t stable_count;
    uint32_t stable_required_count;
    uint32_t steady_window_count;

    APP_FB_TEMP initial_pv;
    APP_FB_TEMP max_pv;
    APP_FB_TEMP min_pv;

    uint32_t saturation_count;

    int64_t steady_error_sum;
    uint32_t steady_error_count;

    APP_FB_BOOL reached_sv;
    APP_FB_BOOL converged;

    APP_FB_AUTOTUNE_REGRESSION_METRICS_T metrics;
} APP_FB_AUTOTUNE_REGRESSION_RUNNER_T;

void app_fb_autotune_regression_init(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    const APP_FB_AUTOTUNE_REGRESSION_CONFIG_T *cfg,
    APP_FB_TEMP initial_pv);

APP_FB_AUTOTUNE_REGRESSION_STATUS_T app_fb_autotune_regression_run(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_TEMP pv,
    APP_FB_PWM pwm);

APP_FB_BOOL app_fb_autotune_regression_get_metrics(
    const APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_AUTOTUNE_REGRESSION_METRICS_T *metrics);

void app_fb_autotune_regression_reset(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_TEMP initial_pv);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* FB_C_AUTOTUNE_REGRESSION_RUNNER_H_ */
