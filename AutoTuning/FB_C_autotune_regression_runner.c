#include "FB_C_autotune_regression_runner.h"

#include <limits.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_regression_abs_i32(int32_t value)
{
    if(value >= 0) return value;
    if(value == INT32_MIN) return INT32_MAX;
    return -value;
}

static APP_FB_BOOL app_fb_regression_config_valid(
    const APP_FB_AUTOTUNE_REGRESSION_CONFIG_T *cfg)
{
    if(cfg == NULL) return APP_FB_FALSE;
    if(cfg->sample_time_ms == 0U) return APP_FB_FALSE;
    if(cfg->max_test_time_ms < cfg->sample_time_ms) return APP_FB_FALSE;
    if(cfg->convergence_band < 0) return APP_FB_FALSE;
    if(cfg->convergence_hold_ms < cfg->sample_time_ms) return APP_FB_FALSE;
    if(cfg->steady_window_ms < cfg->sample_time_ms) return APP_FB_FALSE;
    if(cfg->pwm_min < APP_FB_PWM_MIN) return APP_FB_FALSE;
    if(cfg->pwm_max > APP_FB_PWM_MAX) return APP_FB_FALSE;
    if(cfg->pwm_min >= cfg->pwm_max) return APP_FB_FALSE;
    return APP_FB_TRUE;
}

static uint32_t app_fb_regression_ms_to_samples(uint32_t time_ms, uint32_t sample_time_ms)
{
    uint32_t count;
    if(sample_time_ms == 0U) return 1U;
    count = time_ms / sample_time_ms;
    if((time_ms % sample_time_ms) != 0U) count++;
    if(count == 0U) count = 1U;
    return count;
}

static void app_fb_regression_clear_runtime(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_TEMP initial_pv)
{
    fb->status = APP_FB_AUTOTUNE_REGRESSION_RUNNING;
    fb->sample_count = 0U;
    fb->stable_count = 0U;
    fb->initial_pv = initial_pv;
    fb->max_pv = initial_pv;
    fb->min_pv = initial_pv;
    fb->saturation_count = 0U;
    fb->steady_error_sum = 0;
    fb->steady_error_count = 0U;
    fb->reached_sv = APP_FB_FALSE;
    fb->converged = APP_FB_FALSE;

    fb->metrics.converged = APP_FB_FALSE;
    fb->metrics.overshoot = 0;
    fb->metrics.steady_error = 0;
    fb->metrics.saturation_permille = 0U;
}

static void app_fb_regression_finalize(APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb)
{
    int64_t avg_error;
    int32_t overshoot;
    uint64_t saturation_permille;

    if(fb == NULL) return;

    if(fb->cfg.sv >= fb->initial_pv)
    {
        overshoot = fb->max_pv - fb->cfg.sv;
    }
    else
    {
        overshoot = fb->cfg.sv - fb->min_pv;
    }

    if(overshoot < 0) overshoot = 0;
    fb->metrics.overshoot = overshoot;

    if(fb->steady_error_count > 0U)
    {
        avg_error = fb->steady_error_sum / (int64_t)fb->steady_error_count;
        if(avg_error > INT32_MAX) avg_error = INT32_MAX;
        else if(avg_error < INT32_MIN) avg_error = INT32_MIN;
        fb->metrics.steady_error = (int32_t)avg_error;
    }
    else
    {
        fb->metrics.steady_error = fb->cfg.sv - fb->max_pv;
    }

    if(fb->sample_count > 0U)
    {
        saturation_permille = ((uint64_t)fb->saturation_count * 1000ULL) /
                              (uint64_t)fb->sample_count;
        if(saturation_permille > 1000ULL) saturation_permille = 1000ULL;
        fb->metrics.saturation_permille = (uint16_t)saturation_permille;
    }
    else
    {
        fb->metrics.saturation_permille = 0U;
    }

    fb->metrics.converged = fb->converged;
}

void app_fb_autotune_regression_init(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    const APP_FB_AUTOTUNE_REGRESSION_CONFIG_T *cfg,
    APP_FB_TEMP initial_pv)
{
    if(fb == NULL) return;

    fb->status = APP_FB_AUTOTUNE_REGRESSION_IDLE;

    if(app_fb_regression_config_valid(cfg) == APP_FB_FALSE)
    {
        fb->status = APP_FB_AUTOTUNE_REGRESSION_ERROR;
        return;
    }

    fb->cfg = *cfg;
    fb->stable_required_count = app_fb_regression_ms_to_samples(
        cfg->convergence_hold_ms,
        cfg->sample_time_ms);
    fb->steady_window_count = app_fb_regression_ms_to_samples(
        cfg->steady_window_ms,
        cfg->sample_time_ms);

    app_fb_regression_clear_runtime(fb, initial_pv);
}

void app_fb_autotune_regression_reset(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_TEMP initial_pv)
{
    if(fb == NULL) return;
    if(app_fb_regression_config_valid(&fb->cfg) == APP_FB_FALSE)
    {
        fb->status = APP_FB_AUTOTUNE_REGRESSION_ERROR;
        return;
    }
    app_fb_regression_clear_runtime(fb, initial_pv);
}

APP_FB_AUTOTUNE_REGRESSION_STATUS_T app_fb_autotune_regression_run(
    APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_TEMP pv,
    APP_FB_PWM pwm)
{
    int32_t error;
    int32_t abs_error;
    uint64_t elapsed_ms;
    uint32_t steady_start_sample;

    if(fb == NULL) return APP_FB_AUTOTUNE_REGRESSION_ERROR;
    if(fb->status != APP_FB_AUTOTUNE_REGRESSION_RUNNING)
        return fb->status;

    fb->sample_count++;

    if(pv > fb->max_pv) fb->max_pv = pv;
    if(pv < fb->min_pv) fb->min_pv = pv;

    if(pwm <= fb->cfg.pwm_min || pwm >= fb->cfg.pwm_max)
    {
        if(fb->saturation_count != UINT32_MAX) fb->saturation_count++;
    }

    error = fb->cfg.sv - pv;
    abs_error = app_fb_regression_abs_i32(error);

    if((fb->cfg.sv >= fb->initial_pv && pv >= fb->cfg.sv) ||
       (fb->cfg.sv < fb->initial_pv && pv <= fb->cfg.sv))
    {
        fb->reached_sv = APP_FB_TRUE;
    }

    if(abs_error <= fb->cfg.convergence_band)
    {
        if(fb->stable_count != UINT32_MAX) fb->stable_count++;
    }
    else
    {
        fb->stable_count = 0U;
    }

    if(fb->stable_count >= fb->stable_required_count)
    {
        fb->converged = APP_FB_TRUE;
    }

    /*
     * Measure steady error from a trailing window. To keep this FB streaming
     * and allocation-free, accumulate only after convergence is first reached.
     * The test remains running until at least steady_window_count samples have
     * been collected in the convergence band.
     */
    if(fb->converged == APP_FB_TRUE && abs_error <= fb->cfg.convergence_band)
    {
        if(fb->steady_error_count < fb->steady_window_count)
        {
            fb->steady_error_sum += error;
            fb->steady_error_count++;
        }
    }

    if(fb->converged == APP_FB_TRUE &&
       fb->steady_error_count >= fb->steady_window_count)
    {
        app_fb_regression_finalize(fb);
        fb->status = APP_FB_AUTOTUNE_REGRESSION_DONE;
        return fb->status;
    }

    elapsed_ms = (uint64_t)fb->sample_count * (uint64_t)fb->cfg.sample_time_ms;
    if(elapsed_ms >= (uint64_t)fb->cfg.max_test_time_ms)
    {
        app_fb_regression_finalize(fb);
        fb->status = APP_FB_AUTOTUNE_REGRESSION_TIMEOUT;
        return fb->status;
    }

    /* Silence compiler warnings if future policies use this location. */
    steady_start_sample = 0U;
    (void)steady_start_sample;

    return fb->status;
}

APP_FB_BOOL app_fb_autotune_regression_get_metrics(
    const APP_FB_AUTOTUNE_REGRESSION_RUNNER_T *fb,
    APP_FB_AUTOTUNE_REGRESSION_METRICS_T *metrics)
{
    if(fb == NULL || metrics == NULL) return APP_FB_FALSE;
    if(fb->status != APP_FB_AUTOTUNE_REGRESSION_DONE &&
       fb->status != APP_FB_AUTOTUNE_REGRESSION_TIMEOUT)
    {
        return APP_FB_FALSE;
    }

    *metrics = fb->metrics;
    return APP_FB_TRUE;
}

#ifdef __cplusplus
}
#endif
