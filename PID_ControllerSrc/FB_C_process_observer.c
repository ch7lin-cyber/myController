#include <stdint.h>
#include "FB_C_process_observer.h"

APP_FB_ERROR app_fb_process_observer_init(APP_FB_PROCESS_OBSERVER_T *fb, uint32_t ts, APP_FB_TEMP band, uint32_t settle_ms)
{
    if(!fb) return APP_FB_ERROR_NULL_POINTER;
    if(ts < APP_FB_SAMPLE_TIME_MIN_MS || ts > APP_FB_SAMPLE_TIME_MAX_MS || band < 0 || settle_ms == 0U)
        return APP_FB_ERROR_PARAMETER;
    fb->sample_time_ms = ts;
    fb->settle_band = band;
    fb->settle_required_ms = settle_ms;
    app_fb_process_observer_reset(fb);
    return APP_FB_OK;
}

void app_fb_process_observer_reset(APP_FB_PROCESS_OBSERVER_T *fb)
{
    if(!fb) return;
    fb->previous_pv = 0;
    fb->active_sv = 0;
    fb->elapsed_time_ms = 0U;
    fb->in_band_time_ms = 0U;
    fb->initialized = APP_FB_FALSE;
    fb->metric.peak_pv = 0;
    fb->metric.steady_error = 0;
    fb->metric.overshoot = 0;
    fb->metric.pv_slope_per_s = 0;
    fb->metric.elapsed_time_ms = 0U;
    fb->metric.settling_time_ms = 0U;
    fb->metric.response_active = APP_FB_FALSE;
    fb->metric.settled = APP_FB_FALSE;
}

void app_fb_process_observer_start_response(APP_FB_PROCESS_OBSERVER_T *fb, APP_FB_TEMP sv, APP_FB_TEMP pv)
{
    if(!fb) return;
    fb->previous_pv = pv;
    fb->active_sv = sv;
    fb->elapsed_time_ms = 0U;
    fb->in_band_time_ms = 0U;
    fb->initialized = APP_FB_TRUE;
    fb->metric.peak_pv = pv;
    fb->metric.steady_error = sv - pv;
    fb->metric.overshoot = (pv > sv) ? (pv - sv) : 0;
    fb->metric.pv_slope_per_s = 0;
    fb->metric.elapsed_time_ms = 0U;
    fb->metric.settling_time_ms = 0U;
    fb->metric.response_active = APP_FB_TRUE;
    fb->metric.settled = APP_FB_FALSE;
}

const APP_FB_PROCESS_METRIC_T *app_fb_process_observer_run(APP_FB_PROCESS_OBSERVER_T *fb, APP_FB_TEMP sv, APP_FB_TEMP pv)
{
    int32_t err, delta;
    if(!fb) return 0;

    if(!fb->initialized || sv != fb->active_sv)
        app_fb_process_observer_start_response(fb, sv, pv);

    delta = pv - fb->previous_pv;
    fb->metric.pv_slope_per_s = (int32_t)(((int64_t)delta * 1000LL) / fb->sample_time_ms);

    if(fb->elapsed_time_ms <= UINT32_MAX - fb->sample_time_ms)
        fb->elapsed_time_ms += fb->sample_time_ms;

    if(pv > fb->metric.peak_pv)
        fb->metric.peak_pv = pv;

    fb->metric.steady_error = sv - pv;
    fb->metric.overshoot = (fb->metric.peak_pv > sv) ? (fb->metric.peak_pv - sv) : 0;
    fb->metric.elapsed_time_ms = fb->elapsed_time_ms;

    err = sv - pv;
    if(err < 0) err = -err;

    if(err <= fb->settle_band)
    {
        if(fb->in_band_time_ms <= UINT32_MAX - fb->sample_time_ms)
            fb->in_band_time_ms += fb->sample_time_ms;

        if(!fb->metric.settled && fb->in_band_time_ms >= fb->settle_required_ms)
        {
            fb->metric.settled = APP_FB_TRUE;
            fb->metric.response_active = APP_FB_FALSE;
            fb->metric.settling_time_ms = fb->elapsed_time_ms;
        }
    }
    else
    {
        fb->in_band_time_ms = 0U;

        /*
         * Diagnostics must reflect a disturbance that moves PV out of the
         * settled band. This does not start a new tuning event because the
         * tuner only opens a new event when SV changes / begin_response runs.
         */
        if(fb->metric.settled == APP_FB_TRUE)
        {
            fb->metric.settled = APP_FB_FALSE;
            fb->metric.response_active = APP_FB_TRUE;
        }
    }

    fb->previous_pv = pv;
    return &fb->metric;
}
