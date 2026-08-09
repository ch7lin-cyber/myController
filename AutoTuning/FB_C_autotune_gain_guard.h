#ifndef FB_C_AUTOTUNE_GAIN_GUARD_H_
#define FB_C_AUTOTUNE_GAIN_GUARD_H_

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
    APP_FB_AUTOTUNE_VALIDATION_PASS = 0,
    APP_FB_AUTOTUNE_VALIDATION_RETRY,
    APP_FB_AUTOTUNE_VALIDATION_FAIL
} APP_FB_AUTOTUNE_VALIDATION_STATUS_T;

typedef enum
{
    APP_FB_AUTOTUNE_REJECT_NONE = 0,
    APP_FB_AUTOTUNE_REJECT_NULL_POINTER,
    APP_FB_AUTOTUNE_REJECT_PARAMETER,
    APP_FB_AUTOTUNE_REJECT_NO_CONVERGENCE,
    APP_FB_AUTOTUNE_REJECT_OVERSHOOT,
    APP_FB_AUTOTUNE_REJECT_STEADY_ERROR,
    APP_FB_AUTOTUNE_REJECT_SATURATION,
    APP_FB_AUTOTUNE_REJECT_GAIN_LIMIT,
    APP_FB_AUTOTUNE_REJECT_MIN_SCALE
} APP_FB_AUTOTUNE_REJECT_REASON_T;

typedef struct
{
    /* Absolute Q15 guard limits for the generated PID candidate. */
    int32_t kp_max;
    int32_t ki_max;
    int32_t kd_abs_max;

    /* Candidate scaling policy, percent units. */
    uint16_t initial_scale_percent;
    uint16_t scale_step_percent;
    uint16_t min_scale_percent;

    /* Regression acceptance limits. Temperature unit = controller unit (0.1 degC). */
    int32_t max_overshoot;
    int32_t max_steady_error;
    uint16_t max_saturation_permille;

    /* Preserve application-specific non-gain PID fields. */
    int32_t integral_limit;
    int32_t output_limit;
    int32_t kaw;
} APP_FB_AUTOTUNE_GAIN_GUARD_CONFIG_T;

typedef struct
{
    APP_FB_BOOL converged;
    int32_t overshoot;
    int32_t steady_error;
    uint16_t saturation_permille;
} APP_FB_AUTOTUNE_REGRESSION_METRICS_T;

typedef struct
{
    APP_FB_AUTOTUNE_GAIN_GUARD_CONFIG_T cfg;
    APP_FB_PID_PARAMETER_T raw_pid;
    APP_FB_PID_PARAMETER_T candidate_pid;

    uint16_t current_scale_percent;
    uint16_t attempt_count;

    APP_FB_AUTOTUNE_VALIDATION_STATUS_T status;
    APP_FB_AUTOTUNE_REJECT_REASON_T reject_reason;
} APP_FB_AUTOTUNE_GAIN_GUARD_T;

void app_fb_autotune_gain_guard_init(
    APP_FB_AUTOTUNE_GAIN_GUARD_T *fb,
    const APP_FB_AUTOTUNE_GAIN_GUARD_CONFIG_T *cfg,
    const APP_FB_PID_PARAMETER_T *raw_pid);

APP_FB_BOOL app_fb_autotune_gain_guard_get_candidate(
    const APP_FB_AUTOTUNE_GAIN_GUARD_T *fb,
    APP_FB_PID_PARAMETER_T *candidate_pid);

APP_FB_AUTOTUNE_VALIDATION_STATUS_T app_fb_autotune_gain_guard_evaluate(
    APP_FB_AUTOTUNE_GAIN_GUARD_T *fb,
    const APP_FB_AUTOTUNE_REGRESSION_METRICS_T *metrics);

APP_FB_AUTOTUNE_REJECT_REASON_T app_fb_autotune_gain_guard_get_reject_reason(
    const APP_FB_AUTOTUNE_GAIN_GUARD_T *fb);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* FB_C_AUTOTUNE_GAIN_GUARD_H_ */
