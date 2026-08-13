#ifndef SSM_STD_FB_PROCESS_OBSERVER_H_
#define SSM_STD_FB_PROCESS_OBSERVER_H_

#include <stdint.h>
#include "FB_C_control_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Process response metrics used by the self-tuning layer.
 * Temperature unit follows APP_FB_TEMP (0.1 degC in the current application).
 * The observer does not modify controller parameters directly.
 */
typedef struct
{
    APP_FB_TEMP peak_pv;
    APP_FB_TEMP steady_error;
    APP_FB_TEMP overshoot;
    int32_t pv_slope_per_s;
    uint32_t elapsed_time_ms;
    uint32_t settling_time_ms;
    APP_FB_BOOL response_active;
    APP_FB_BOOL settled;
} APP_FB_PROCESS_METRIC_T;

typedef struct
{
    uint32_t sample_time_ms;
    APP_FB_TEMP settle_band;
    uint32_t settle_required_ms;
    APP_FB_TEMP previous_pv;
    APP_FB_TEMP active_sv;
    uint32_t elapsed_time_ms;
    uint32_t in_band_time_ms;
    APP_FB_BOOL initialized;
    APP_FB_PROCESS_METRIC_T metric;
} APP_FB_PROCESS_OBSERVER_T;

APP_FB_ERROR app_fb_process_observer_init(
    APP_FB_PROCESS_OBSERVER_T *fb,
    uint32_t sample_time_ms,
    APP_FB_TEMP settle_band,
    uint32_t settle_required_ms);

void app_fb_process_observer_reset(APP_FB_PROCESS_OBSERVER_T *fb);

void app_fb_process_observer_start_response(
    APP_FB_PROCESS_OBSERVER_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv);

const APP_FB_PROCESS_METRIC_T *app_fb_process_observer_run(
    APP_FB_PROCESS_OBSERVER_T *fb,
    APP_FB_TEMP sv,
    APP_FB_TEMP pv);

#ifdef __cplusplus
}
#endif

#endif
