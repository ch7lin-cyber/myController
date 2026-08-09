#include "FB_C_relay_auto_tune.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FB_AUTOTUNE_PI                (3.14159265358979323846)
#define APP_FB_AUTOTUNE_Q15               (32768.0)
#define APP_FB_AUTOTUNE_MIN_AMPLITUDE     (1.0)
#define APP_FB_AUTOTUNE_MAX_I_LIMIT       (1000000)

static int32_t app_fb_autotune_clamp_i32(int64_t value, int32_t min_value, int32_t max_value)
{
    if(value < (int64_t)min_value) return min_value;
    if(value > (int64_t)max_value) return max_value;
    return (int32_t)value;
}

static APP_FB_PWM app_fb_autotune_clamp_pwm(const APP_FB_RELAY_AUTOTUNE_T *fb, int32_t value)
{
    if(value < fb->cfg.pwm_min) return fb->cfg.pwm_min;
    if(value > fb->cfg.pwm_max) return fb->cfg.pwm_max;
    return value;
}

static APP_FB_BOOL app_fb_autotune_config_valid(const APP_FB_RELAY_AUTOTUNE_CONFIG_T *cfg)
{
    int64_t low_pwm;
    int64_t high_pwm;

    if(cfg == NULL) return APP_FB_FALSE;
    if(cfg->sample_time_ms == 0U) return APP_FB_FALSE;
    if(cfg->max_test_time_ms < cfg->sample_time_ms) return APP_FB_FALSE;
    if(cfg->min_cycles < 2U) return APP_FB_FALSE;
    if(cfg->hysteresis <= 0) return APP_FB_FALSE;
    if(cfg->relay_amplitude <= 0) return APP_FB_FALSE;
    if(cfg->pwm_min < APP_FB_PWM_MIN || cfg->pwm_max > APP_FB_PWM_MAX) return APP_FB_FALSE;
    if(cfg->pwm_min >= cfg->pwm_max) return APP_FB_FALSE;
    if(cfg->pv_min >= cfg->pv_max) return APP_FB_FALSE;
    if(cfg->sv <= cfg->pv_min || cfg->sv >= cfg->pv_max) return APP_FB_FALSE;

    low_pwm = (int64_t)cfg->bias_pwm - (int64_t)cfg->relay_amplitude;
    high_pwm = (int64_t)cfg->bias_pwm + (int64_t)cfg->relay_amplitude;
    if(low_pwm < cfg->pwm_min || high_pwm > cfg->pwm_max) return APP_FB_FALSE;

    return APP_FB_TRUE;
}

static void app_fb_autotune_reset_measurement(APP_FB_RELAY_AUTOTUNE_T *fb, APP_FB_TEMP pv)
{
    fb->sample_count = 0U;
    fb->last_high_to_low_sample = 0U;
    fb->period_sample_sum = 0U;
    fb->period_count = 0U;
    fb->segment_max = pv;
    fb->segment_min = pv;
    fb->peak_sum = 0;
    fb->trough_sum = 0;
    fb->peak_count = 0U;
    fb->trough_count = 0U;
    fb->first_high_to_low_seen = APP_FB_FALSE;
}

static int32_t app_fb_autotune_round_q15(double gain)
{
    double scaled = gain * APP_FB_AUTOTUNE_Q15;
    if(scaled >= (double)INT32_MAX) return INT32_MAX;
    if(scaled <= (double)INT32_MIN) return INT32_MIN;
    return (int32_t)llround(scaled);
}

static void app_fb_autotune_build_pid(APP_FB_RELAY_AUTOTUNE_T *fb)
{
    double kp;
    double ti;
    double td;
    double ts_s;
    double ki_discrete;
    double kd_discrete;
    int32_t ki_q15;
    int64_t integral_limit;

    ts_s = (double)fb->cfg.sample_time_ms / 1000.0;

    if(fb->cfg.rule == APP_FB_AUTOTUNE_RULE_PID_CONSERVATIVE)
    {
        /* Tyreus-Luyben style conservative PID. */
        kp = 0.45 * fb->result.ku;
        ti = 2.20 * fb->result.pu_s;
        td = fb->result.pu_s / 6.30;
    }
    else
    {
        /* Tyreus-Luyben style conservative PI. */
        kp = 0.31 * fb->result.ku;
        ti = 2.20 * fb->result.pu_s;
        td = 0.0;
    }

    if(ti <= 0.0) ti = ts_s;

    ki_discrete = kp * ts_s / ti;

    /*
     * Current controller differentiates PV as pv[k]-pv[k-1] and then adds
     * kd*dPV to the output. Standard derivative-on-measurement damping must
     * therefore use a negative discrete Kd.
     */
    kd_discrete = (td > 0.0) ? -(kp * td / ts_s) : 0.0;

    fb->result.pid.kp = app_fb_autotune_round_q15(kp);
    fb->result.pid.ki = app_fb_autotune_round_q15(ki_discrete);
    fb->result.pid.kd = app_fb_autotune_round_q15(kd_discrete);

    ki_q15 = fb->result.pid.ki;
    if(ki_q15 > 0)
    {
        integral_limit = ((int64_t)APP_FB_PWM_MAX * (int64_t)APP_FB_Q15_ONE) / ki_q15;
        fb->result.pid.integral_limit = app_fb_autotune_clamp_i32(
            integral_limit, 1000, APP_FB_AUTOTUNE_MAX_I_LIMIT);
    }
    else
    {
        fb->result.pid.integral_limit = 32767;
    }

    fb->result.pid.output_limit = APP_FB_PWM_MAX;
    fb->result.pid.kaw = APP_FB_PID_KAW_DEFAULT;
}

static void app_fb_autotune_finalize(APP_FB_RELAY_AUTOTUNE_T *fb)
{
    double avg_peak;
    double avg_trough;
    double amplitude;
    double avg_period_samples;
    double pu_s;
    double d;

    if(fb->period_count < fb->cfg.min_cycles ||
       fb->peak_count == 0U || fb->trough_count == 0U)
    {
        fb->status = APP_FB_AUTOTUNE_ERROR;
        fb->error = APP_FB_AUTOTUNE_ERROR_NO_OSCILLATION;
        fb->output_pwm = fb->cfg.bias_pwm;
        return;
    }

    avg_peak = (double)fb->peak_sum / (double)fb->peak_count;
    avg_trough = (double)fb->trough_sum / (double)fb->trough_count;
    amplitude = (avg_peak - avg_trough) * 0.5;

    if(amplitude < APP_FB_AUTOTUNE_MIN_AMPLITUDE)
    {
        fb->status = APP_FB_AUTOTUNE_ERROR;
        fb->error = APP_FB_AUTOTUNE_ERROR_NO_OSCILLATION;
        fb->output_pwm = fb->cfg.bias_pwm;
        return;
    }

    avg_period_samples = (double)fb->period_sample_sum / (double)fb->period_count;
    pu_s = avg_period_samples * ((double)fb->cfg.sample_time_ms / 1000.0);
    d = (double)fb->cfg.relay_amplitude;

    fb->result.pv_amplitude = amplitude;
    fb->result.relay_amplitude = fb->cfg.relay_amplitude;
    fb->result.pu_s = pu_s;
    fb->result.ku = (4.0 * d) / (APP_FB_AUTOTUNE_PI * amplitude);

    app_fb_autotune_build_pid(fb);

    fb->status = APP_FB_AUTOTUNE_DONE;
    fb->error = APP_FB_AUTOTUNE_ERROR_NONE;
    fb->output_pwm = fb->cfg.bias_pwm;
}

void app_fb_relay_autotune_init(
    APP_FB_RELAY_AUTOTUNE_T *fb,
    const APP_FB_RELAY_AUTOTUNE_CONFIG_T *cfg)
{
    if(fb == NULL) return;

    fb->status = APP_FB_AUTOTUNE_IDLE;
    fb->error = APP_FB_AUTOTUNE_ERROR_NONE;
    fb->output_pwm = 0;
    fb->relay_high = APP_FB_FALSE;
    fb->result.ku = 0.0;
    fb->result.pu_s = 0.0;
    fb->result.pv_amplitude = 0.0;
    fb->result.relay_amplitude = 0;
    fb->result.pid.kp = 0;
    fb->result.pid.ki = 0;
    fb->result.pid.kd = 0;
    fb->result.pid.integral_limit = 0;
    fb->result.pid.output_limit = 0;
    fb->result.pid.kaw = APP_FB_PID_KAW_DEFAULT;

    if(cfg == NULL)
    {
        fb->error = APP_FB_AUTOTUNE_ERROR_NULL_POINTER;
        fb->status = APP_FB_AUTOTUNE_ERROR;
        return;
    }

    fb->cfg = *cfg;

    if(app_fb_autotune_config_valid(cfg) == APP_FB_FALSE)
    {
        fb->error = APP_FB_AUTOTUNE_ERROR_PARAMETER;
        fb->status = APP_FB_AUTOTUNE_ERROR;
    }
}

void app_fb_relay_autotune_start(
    APP_FB_RELAY_AUTOTUNE_T *fb,
    APP_FB_TEMP initial_pv)
{
    if(fb == NULL) return;
    if(app_fb_autotune_config_valid(&fb->cfg) == APP_FB_FALSE)
    {
        fb->status = APP_FB_AUTOTUNE_ERROR;
        fb->error = APP_FB_AUTOTUNE_ERROR_PARAMETER;
        return;
    }
    if(initial_pv <= fb->cfg.pv_min || initial_pv >= fb->cfg.pv_max)
    {
        fb->status = APP_FB_AUTOTUNE_ERROR;
        fb->error = APP_FB_AUTOTUNE_ERROR_SAFETY_LIMIT;
        fb->output_pwm = fb->cfg.bias_pwm;
        return;
    }

    app_fb_autotune_reset_measurement(fb, initial_pv);

    fb->relay_high = (initial_pv <= fb->cfg.sv) ? APP_FB_TRUE : APP_FB_FALSE;
    fb->output_pwm = app_fb_autotune_clamp_pwm(
        fb,
        fb->relay_high ?
            (fb->cfg.bias_pwm + fb->cfg.relay_amplitude) :
            (fb->cfg.bias_pwm - fb->cfg.relay_amplitude));

    fb->status = APP_FB_AUTOTUNE_RUNNING;
    fb->error = APP_FB_AUTOTUNE_ERROR_NONE;
}

APP_FB_PWM app_fb_relay_autotune_run(
    APP_FB_RELAY_AUTOTUNE_T *fb,
    APP_FB_TEMP pv)
{
    uint64_t elapsed_ms;

    if(fb == NULL) return 0;
    if(fb->status != APP_FB_AUTOTUNE_RUNNING) return fb->output_pwm;

    if(pv <= fb->cfg.pv_min || pv >= fb->cfg.pv_max)
    {
        fb->status = APP_FB_AUTOTUNE_ERROR;
        fb->error = APP_FB_AUTOTUNE_ERROR_SAFETY_LIMIT;
        fb->output_pwm = fb->cfg.bias_pwm;
        return fb->output_pwm;
    }

    if(pv > fb->segment_max) fb->segment_max = pv;
    if(pv < fb->segment_min) fb->segment_min = pv;

    fb->sample_count++;
    elapsed_ms = (uint64_t)fb->sample_count * (uint64_t)fb->cfg.sample_time_ms;
    if(elapsed_ms >= (uint64_t)fb->cfg.max_test_time_ms)
    {
        fb->status = APP_FB_AUTOTUNE_ERROR;
        fb->error = APP_FB_AUTOTUNE_ERROR_TIMEOUT;
        fb->output_pwm = fb->cfg.bias_pwm;
        return fb->output_pwm;
    }

    if(fb->relay_high == APP_FB_TRUE)
    {
        if(pv >= (fb->cfg.sv + fb->cfg.hysteresis))
        {
            fb->peak_sum += fb->segment_max;
            fb->peak_count++;

            if(fb->first_high_to_low_seen == APP_FB_TRUE)
            {
                uint32_t period_samples = fb->sample_count - fb->last_high_to_low_sample;
                if(period_samples > 0U)
                {
                    fb->period_sample_sum += period_samples;
                    fb->period_count++;
                }
            }
            else
            {
                fb->first_high_to_low_seen = APP_FB_TRUE;
            }

            fb->last_high_to_low_sample = fb->sample_count;
            fb->relay_high = APP_FB_FALSE;
            fb->segment_max = pv;
            fb->segment_min = pv;
        }
    }
    else
    {
        if(pv <= (fb->cfg.sv - fb->cfg.hysteresis))
        {
            fb->trough_sum += fb->segment_min;
            fb->trough_count++;
            fb->relay_high = APP_FB_TRUE;
            fb->segment_max = pv;
            fb->segment_min = pv;
        }
    }

    if(fb->period_count >= fb->cfg.min_cycles &&
       fb->peak_count >= fb->cfg.min_cycles &&
       fb->trough_count >= fb->cfg.min_cycles)
    {
        app_fb_autotune_finalize(fb);
        return fb->output_pwm;
    }

    fb->output_pwm = app_fb_autotune_clamp_pwm(
        fb,
        fb->relay_high ?
            (fb->cfg.bias_pwm + fb->cfg.relay_amplitude) :
            (fb->cfg.bias_pwm - fb->cfg.relay_amplitude));

    return fb->output_pwm;
}

void app_fb_relay_autotune_abort(APP_FB_RELAY_AUTOTUNE_T *fb)
{
    if(fb == NULL) return;
    fb->status = APP_FB_AUTOTUNE_IDLE;
    fb->error = APP_FB_AUTOTUNE_ERROR_NONE;
    fb->output_pwm = fb->cfg.bias_pwm;
}

APP_FB_AUTOTUNE_STATUS_T app_fb_relay_autotune_get_status(
    const APP_FB_RELAY_AUTOTUNE_T *fb)
{
    if(fb == NULL) return APP_FB_AUTOTUNE_ERROR;
    return fb->status;
}

APP_FB_AUTOTUNE_ERROR_T app_fb_relay_autotune_get_error(
    const APP_FB_RELAY_AUTOTUNE_T *fb)
{
    if(fb == NULL) return APP_FB_AUTOTUNE_ERROR_NULL_POINTER;
    return fb->error;
}

APP_FB_BOOL app_fb_relay_autotune_get_result(
    const APP_FB_RELAY_AUTOTUNE_T *fb,
    APP_FB_RELAY_AUTOTUNE_RESULT_T *result)
{
    if(fb == NULL || result == NULL) return APP_FB_FALSE;
    if(fb->status != APP_FB_AUTOTUNE_DONE) return APP_FB_FALSE;
    *result = fb->result;
    return APP_FB_TRUE;
}

#ifdef __cplusplus
}
#endif
