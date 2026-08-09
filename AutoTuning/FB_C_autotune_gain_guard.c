#include "FB_C_autotune_gain_guard.h"

#include <limits.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_autotune_abs_i32(int32_t value)
{
    if(value == INT32_MIN) return INT32_MAX;
    return (value < 0) ? -value : value;
}

static int32_t app_fb_autotune_scale_i32(int32_t value, uint16_t percent)
{
    int64_t scaled = ((int64_t)value * (int64_t)percent) / 100;
    if(scaled > INT32_MAX) return INT32_MAX;
    if(scaled < INT32_MIN) return INT32_MIN;
    return (int32_t)scaled;
}

static APP_FB_BOOL app_fb_autotune_gain_guard_config_valid(
    const APP_FB_AUTOTUNE_GAIN_GUARD_CONFIG_T *cfg)
{
    if(cfg == NULL) return APP_FB_FALSE;
    if(cfg->kp_max <= 0) return APP_FB_FALSE;
    if(cfg->ki_max < 0) return APP_FB_FALSE;
    if(cfg->kd_abs_max < 0) return APP_FB_FALSE;
    if(cfg->initial_scale_percent == 0U || cfg->initial_scale_percent > 100U) return APP_FB_FALSE;
    if(cfg->scale_step_percent == 0U || cfg->scale_step_percent >= 100U) return APP_FB_FALSE;
    if(cfg->min_scale_percent == 0U || cfg->min_scale_percent > cfg->initial_scale_percent) return APP_FB_FALSE;
    if(cfg->max_overshoot < 0) return APP_FB_FALSE;
    if(cfg->max_steady_error < 0) return APP_FB_FALSE;
    if(cfg->max_saturation_permille > 1000U) return APP_FB_FALSE;
    if(cfg->integral_limit <= 0) return APP_FB_FALSE;
    if(cfg->output_limit <= 0) return APP_FB_FALSE;
    return APP_FB_TRUE;
}

static APP_FB_BOOL app_fb_autotune_raw_pid_within_limits(
    const APP_FB_AUTOTUNE_GAIN_GUARD_T *fb)
{
    if(fb->raw_pid.kp < 0 || fb->raw_pid.kp > fb->cfg.kp_max) return APP_FB_FALSE;
    if(fb->raw_pid.ki < 0 || fb->raw_pid.ki > fb->cfg.ki_max) return APP_FB_FALSE;
    if(app_fb_autotune_abs_i32(fb->raw_pid.kd) > fb->cfg.kd_abs_max) return APP_FB_FALSE;
    return APP_FB_TRUE;
}

static void app_fb_autotune_build_candidate(APP_FB_AUTOTUNE_GAIN_GUARD_T *fb)
{
    int32_t kp;
    int32_t ki;
    int32_t kd;

    kp = app_fb_autotune_scale_i32(fb->raw_pid.kp, fb->current_scale_percent);
    ki = app_fb_autotune_scale_i32(fb->raw_pid.ki, fb->current_scale_percent);
    kd = app_fb_autotune_scale_i32(fb->raw_pid.kd, fb->current_scale_percent);

    if(kp > fb->cfg.kp_max) kp = fb->cfg.kp_max;
    if(ki > fb->cfg.ki_max) ki = fb->cfg.ki_max;

    if(kd > fb->cfg.kd_abs_max) kd = fb->cfg.kd_abs_max;
    else if(kd < -fb->cfg.kd_abs_max) kd = -fb->cfg.kd_abs_max;

    fb->candidate_pid.kp = kp;
    fb->candidate_pid.ki = ki;
    fb->candidate_pid.kd = kd;
    fb->candidate_pid.integral_limit = fb->cfg.integral_limit;
    fb->candidate_pid.output_limit = fb->cfg.output_limit;
    fb->candidate_pid.kaw = fb->cfg.kaw;
}

static APP_FB_BOOL app_fb_autotune_reduce_scale(APP_FB_AUTOTUNE_GAIN_GUARD_T *fb)
{
    uint16_t next_scale;

    if(fb->current_scale_percent <= fb->cfg.min_scale_percent)
        return APP_FB_FALSE;

    if(fb->current_scale_percent <= fb->cfg.scale_step_percent)
        next_scale = fb->cfg.min_scale_percent;
    else
        next_scale = (uint16_t)(fb->current_scale_percent - fb->cfg.scale_step_percent);

    if(next_scale < fb->cfg.min_scale_percent)
        next_scale = fb->cfg.min_scale_percent;

    if(next_scale == fb->current_scale_percent)
        return APP_FB_FALSE;

    fb->current_scale_percent = next_scale;
    fb->attempt_count++;
    app_fb_autotune_build_candidate(fb);
    return APP_FB_TRUE;
}

void app_fb_autotune_gain_guard_init(
    APP_FB_AUTOTUNE_GAIN_GUARD_T *fb,
    const APP_FB_AUTOTUNE_GAIN_GUARD_CONFIG_T *cfg,
    const APP_FB_PID_PARAMETER_T *raw_pid)
{
    if(fb == NULL) return;

    fb->status = APP_FB_AUTOTUNE_VALIDATION_FAIL;
    fb->reject_reason = APP_FB_AUTOTUNE_REJECT_NONE;
    fb->current_scale_percent = 0U;
    fb->attempt_count = 0U;

    if(cfg == NULL || raw_pid == NULL)
    {
        fb->reject_reason = APP_FB_AUTOTUNE_REJECT_NULL_POINTER;
        return;
    }

    fb->cfg = *cfg;
    fb->raw_pid = *raw_pid;

    if(app_fb_autotune_gain_guard_config_valid(cfg) == APP_FB_FALSE)
    {
        fb->reject_reason = APP_FB_AUTOTUNE_REJECT_PARAMETER;
        return;
    }

    fb->current_scale_percent = cfg->initial_scale_percent;
    fb->attempt_count = 1U;

    /*
     * Raw relay tuning may be much more aggressive than a thermal plant can
     * safely use. Absolute limits are therefore applied before regression.
     */
    app_fb_autotune_build_candidate(fb);

    if(app_fb_autotune_raw_pid_within_limits(fb) == APP_FB_FALSE)
        fb->reject_reason = APP_FB_AUTOTUNE_REJECT_GAIN_LIMIT;

    fb->status = APP_FB_AUTOTUNE_VALIDATION_RETRY;
}

APP_FB_BOOL app_fb_autotune_gain_guard_get_candidate(
    const APP_FB_AUTOTUNE_GAIN_GUARD_T *fb,
    APP_FB_PID_PARAMETER_T *candidate_pid)
{
    if(fb == NULL || candidate_pid == NULL) return APP_FB_FALSE;
    if(fb->status == APP_FB_AUTOTUNE_VALIDATION_FAIL) return APP_FB_FALSE;
    *candidate_pid = fb->candidate_pid;
    return APP_FB_TRUE;
}

APP_FB_AUTOTUNE_VALIDATION_STATUS_T app_fb_autotune_gain_guard_evaluate(
    APP_FB_AUTOTUNE_GAIN_GUARD_T *fb,
    const APP_FB_AUTOTUNE_REGRESSION_METRICS_T *metrics)
{
    int32_t abs_steady_error;

    if(fb == NULL || metrics == NULL)
        return APP_FB_AUTOTUNE_VALIDATION_FAIL;

    if(fb->status == APP_FB_AUTOTUNE_VALIDATION_FAIL)
        return fb->status;

    fb->reject_reason = APP_FB_AUTOTUNE_REJECT_NONE;

    if(metrics->converged == APP_FB_FALSE)
        fb->reject_reason = APP_FB_AUTOTUNE_REJECT_NO_CONVERGENCE;
    else if(metrics->overshoot > fb->cfg.max_overshoot)
        fb->reject_reason = APP_FB_AUTOTUNE_REJECT_OVERSHOOT;
    else
    {
        abs_steady_error = app_fb_autotune_abs_i32(metrics->steady_error);
        if(abs_steady_error > fb->cfg.max_steady_error)
            fb->reject_reason = APP_FB_AUTOTUNE_REJECT_STEADY_ERROR;
        else if(metrics->saturation_permille > fb->cfg.max_saturation_permille)
            fb->reject_reason = APP_FB_AUTOTUNE_REJECT_SATURATION;
    }

    if(fb->reject_reason == APP_FB_AUTOTUNE_REJECT_NONE)
    {
        fb->status = APP_FB_AUTOTUNE_VALIDATION_PASS;
        return fb->status;
    }

    if(app_fb_autotune_reduce_scale(fb) == APP_FB_TRUE)
    {
        fb->status = APP_FB_AUTOTUNE_VALIDATION_RETRY;
        return fb->status;
    }

    fb->reject_reason = APP_FB_AUTOTUNE_REJECT_MIN_SCALE;
    fb->status = APP_FB_AUTOTUNE_VALIDATION_FAIL;
    return fb->status;
}

APP_FB_AUTOTUNE_REJECT_REASON_T app_fb_autotune_gain_guard_get_reject_reason(
    const APP_FB_AUTOTUNE_GAIN_GUARD_T *fb)
{
    if(fb == NULL) return APP_FB_AUTOTUNE_REJECT_NULL_POINTER;
    return fb->reject_reason;
}

#ifdef __cplusplus
}
#endif
