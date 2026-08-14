#ifndef SSM_STD_FB_SELF_TUNER_H_
#define SSM_STD_FB_SELF_TUNER_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_process_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_FB_SELF_TUNE_REASON_NONE = 0,
    APP_FB_SELF_TUNE_REASON_KI_INCREASE,
    APP_FB_SELF_TUNE_REASON_KI_DECREASE,
    APP_FB_SELF_TUNE_REASON_PREDICTIVE_TIME_INCREASE,
    APP_FB_SELF_TUNE_REASON_PREDICTIVE_TIME_DECREASE,
    APP_FB_SELF_TUNE_REASON_MULTIPLE
} APP_FB_SELF_TUNE_REASON_T;

typedef struct
{
    int32_t ki_min;
    int32_t ki_max;
    int32_t ki_step;
    uint32_t predictive_time_min_ms;
    uint32_t predictive_time_max_ms;
    uint32_t predictive_time_step_ms;
    APP_FB_TEMP steady_error_threshold;
    APP_FB_TEMP overshoot_threshold;
} APP_FB_SELF_TUNER_PARAMETER_T;

typedef struct
{
    APP_FB_BOOL enable;
    APP_FB_BOOL update_ready;
    APP_FB_BOOL settled_consumed;
    int32_t suggested_ki;
    uint32_t suggested_predictive_time_ms;
    uint32_t tune_count;
    APP_FB_SELF_TUNE_REASON_T last_tune_reason;
    APP_FB_SELF_TUNER_PARAMETER_T param;
} APP_FB_SELF_TUNER_T;

APP_FB_ERROR app_fb_self_tuner_init(
    APP_FB_SELF_TUNER_T *fb,
    const APP_FB_SELF_TUNER_PARAMETER_T *param,
    int32_t initial_ki,
    uint32_t initial_predictive_time_ms);

void app_fb_self_tuner_reset(
    APP_FB_SELF_TUNER_T *fb,
    int32_t initial_ki,
    uint32_t initial_predictive_time_ms);

void app_fb_self_tuner_set_enable(
    APP_FB_SELF_TUNER_T *fb,
    APP_FB_BOOL enable);

APP_FB_BOOL app_fb_self_tuner_run(
    APP_FB_SELF_TUNER_T *fb,
    const APP_FB_PROCESS_METRIC_T *metric);

#ifdef __cplusplus
}
#endif
#endif
