#include "FB_C_feedforward_learning.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_ff_learning_limit(APP_FB_FF_LEARNING_T *fb, int32_t value)
{
    if(value > fb->offset_limit) return fb->offset_limit;
    if(value < -fb->offset_limit) return -fb->offset_limit;
    return value;
}

static int32_t app_fb_ff_clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static uint32_t app_fb_ff_clamp_u32(uint32_t value, uint32_t max_value)
{
    return (value > max_value) ? max_value : value;
}

static int64_t app_fb_ff_abs_i32(int32_t value)
{
    if(value >= 0) return (int64_t)value;
    return -(int64_t)value;
}

static int64_t app_fb_ff_abs_i64(int64_t value)
{
    return (value >= 0) ? value : -value;
}

static uint32_t app_fb_ff_time_to_count_ceil(uint32_t time_ms, uint32_t sample_time_ms)
{
    uint64_t numerator;
    uint64_t count;

    if(time_ms == 0U) return 0U;
    if(sample_time_ms == 0U) return 0U;

    numerator = (uint64_t)time_ms + (uint64_t)sample_time_ms - 1ULL;
    count = numerator / (uint64_t)sample_time_ms;

    if(count == 0ULL) count = 1ULL;
    if(count > (uint64_t)UINT32_MAX) count = (uint64_t)UINT32_MAX;
    return (uint32_t)count;
}

static void app_fb_ff_learning_clear_window(APP_FB_FF_LEARNING_T *fb)
{
    fb->stable_counter = 0U;
    fb->pid_sum = 0;
    fb->pid_count = 0U;
}

static void app_fb_ff_learning_clear_fraction(APP_FB_FF_LEARNING_T *fb)
{
    fb->learn_accumulator = 0;
}

static APP_FB_ERROR app_fb_ff_learning_apply_parameter(
    APP_FB_FF_LEARNING_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *param,
    uint32_t sample_time_ms)
{
    int32_t error_threshold = APP_FB_ADAPTIVE_ERROR;
    int32_t gain = APP_FB_ADAPTIVE_GAIN;
    int32_t sv_change_threshold = APP_FB_ADAPTIVE_SV_CHANGE;
    int32_t pid_deadband = APP_FB_ADAPTIVE_PID_DEADBAND;
    uint32_t stable_time_ms = APP_FB_ADAPTIVE_STABLE_TIME_MS;
    uint32_t freeze_time_ms = APP_FB_ADAPTIVE_FREEZE_TIME_MS;
    int32_t offset_limit = APP_FB_ADAPTIVE_OFFSET_LIMIT;

    if(fb == 0) return APP_FB_ERROR_NULL_POINTER;
    if(sample_time_ms < APP_FB_SAMPLE_TIME_MIN_MS ||
       sample_time_ms > APP_FB_SAMPLE_TIME_MAX_MS)
        return APP_FB_ERROR_PARAMETER;

    if(param != 0)
    {
        error_threshold = app_fb_ff_clamp_i32(param->error_threshold, 0, APP_FB_ADAPTIVE_ERROR_MAX);
        gain = app_fb_ff_clamp_i32(param->gain, 0, APP_FB_ADAPTIVE_GAIN_MAX);
        sv_change_threshold = app_fb_ff_clamp_i32(param->sv_change_threshold, APP_FB_ADAPTIVE_SV_CHANGE_MIN, APP_FB_ADAPTIVE_SV_CHANGE_MAX);
        pid_deadband = app_fb_ff_clamp_i32(param->pid_deadband, 0, APP_FB_ADAPTIVE_PID_DEADBAND_MAX);
        stable_time_ms = app_fb_ff_clamp_u32(param->stable_time_ms, APP_FB_ADAPTIVE_TIME_MAX_MS);
        freeze_time_ms = app_fb_ff_clamp_u32(param->freeze_time_ms, APP_FB_ADAPTIVE_TIME_MAX_MS);
        offset_limit = app_fb_ff_clamp_i32(param->offset_limit, 0, APP_FB_ADAPTIVE_OFFSET_LIMIT_MAX);
    }

    /* stable_time=0 is normalized to one cycle so a learning window can never
       complete with zero evidence. freeze_time=0 intentionally disables freeze. */
    if(stable_time_ms == 0U)
        stable_time_ms = sample_time_ms;

    fb->error_threshold = error_threshold;
    fb->gain = gain;
    fb->sv_change_threshold = sv_change_threshold;
    fb->pid_deadband = pid_deadband;
    fb->sample_time_ms = sample_time_ms;
    fb->stable_time_ms = stable_time_ms;
    fb->freeze_time_ms = freeze_time_ms;
    fb->stable_count = app_fb_ff_time_to_count_ceil(stable_time_ms, sample_time_ms);
    fb->freeze_count = app_fb_ff_time_to_count_ceil(freeze_time_ms, sample_time_ms);
    fb->offset_limit = offset_limit;

    if(fb->stable_count == 0U)
        fb->stable_count = 1U;

    return APP_FB_OK;
}

void app_fb_ff_learning_init(APP_FB_FF_LEARNING_T *fb, const APP_FB_ADAPTIVE_PARAMETER_T *param)
{
    (void)app_fb_ff_learning_init_timed(
        fb,
        param,
        APP_FB_SAMPLE_TIME_DEFAULT_MS);
}

APP_FB_ERROR app_fb_ff_learning_init_timed(
    APP_FB_FF_LEARNING_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *param,
    uint32_t sample_time_ms)
{
    APP_FB_ERROR status;

    if(fb == 0) return APP_FB_ERROR_NULL_POINTER;

    status = app_fb_ff_learning_apply_parameter(fb, param, sample_time_ms);
    if(status != APP_FB_OK) return status;

    fb->counter = 0U;
    fb->offset = 0;
    fb->pid_sum = 0;
    fb->pid_count = 0U;
    fb->stable_counter = 0U;
    fb->freeze_counter = 0U;
    fb->previous_sv = 0;
    fb->sv_initialized = APP_FB_FALSE;
    fb->learn_accumulator = 0;
    return APP_FB_OK;
}

void app_fb_ff_learning_reconfigure(APP_FB_FF_LEARNING_T *fb, const APP_FB_ADAPTIVE_PARAMETER_T *param)
{
    uint32_t sample_time_ms;

    if(fb == 0) return;
    sample_time_ms = fb->sample_time_ms;
    if(sample_time_ms == 0U)
        sample_time_ms = APP_FB_SAMPLE_TIME_DEFAULT_MS;

    (void)app_fb_ff_learning_reconfigure_timed(fb, param, sample_time_ms);
}

APP_FB_ERROR app_fb_ff_learning_reconfigure_timed(
    APP_FB_FF_LEARNING_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *param,
    uint32_t sample_time_ms)
{
    APP_FB_ERROR status;

    if(fb == 0) return APP_FB_ERROR_NULL_POINTER;

    status = app_fb_ff_learning_apply_parameter(fb, param, sample_time_ms);
    if(status != APP_FB_OK) return status;

    /* Runtime policy: keep learned offset, but clamp it to the new limit. */
    fb->offset = app_fb_ff_learning_limit(fb, fb->offset);

    /* Discard partial evidence collected under the previous configuration. */
    app_fb_ff_learning_clear_fraction(fb);
    app_fb_ff_learning_clear_window(fb);

    /* Restart the settling period only when an SV context already exists. */
    if(fb->sv_initialized == APP_FB_TRUE)
        fb->freeze_counter = fb->freeze_count;
    else
        fb->freeze_counter = 0U;

    return APP_FB_OK;
}

void app_fb_ff_learning_reset(APP_FB_FF_LEARNING_T *fb)
{
    if(fb == 0) return;
    fb->offset = 0;
    fb->counter = 0U;
    fb->previous_sv = 0;
    fb->sv_initialized = APP_FB_FALSE;
    fb->freeze_counter = 0U;
    app_fb_ff_learning_clear_fraction(fb);
    app_fb_ff_learning_clear_window(fb);
}

int32_t app_fb_ff_learning_run(APP_FB_FF_LEARNING_T *fb, int32_t sv, int32_t pv, int32_t pid_output, APP_FB_BOOL allow_learning)
{
    int32_t error;
    int32_t avg_pid;
    int64_t learn_q15;
    int64_t new_offset;
    int64_t sv_delta;

    if(fb == 0) return 0;

    if(fb->sv_initialized == APP_FB_FALSE)
    {
        fb->previous_sv = sv;
        fb->sv_initialized = APP_FB_TRUE;
        fb->freeze_counter = fb->freeze_count;
        app_fb_ff_learning_clear_fraction(fb);
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    sv_delta = (int64_t)sv - (int64_t)fb->previous_sv;
    if(app_fb_ff_abs_i64(sv_delta) >= (int64_t)fb->sv_change_threshold)
    {
        fb->previous_sv = sv;
        fb->freeze_counter = fb->freeze_count;
        app_fb_ff_learning_clear_fraction(fb);
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    if(fb->freeze_counter > 0U)
    {
        fb->freeze_counter--;
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    if(allow_learning == APP_FB_FALSE)
    {
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    error = sv - pv;
    if(app_fb_ff_abs_i32(error) > (int64_t)fb->error_threshold)
    {
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    if(app_fb_ff_abs_i32(pid_output) > (int64_t)fb->pid_deadband)
    {
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    if(fb->stable_counter != UINT32_MAX)
        fb->stable_counter++;
    fb->pid_sum += (int64_t)pid_output;
    if(fb->pid_count != UINT32_MAX)
        fb->pid_count++;

    if(fb->stable_counter < fb->stable_count) return fb->offset;

    if(fb->pid_count == 0U)
    {
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    avg_pid = (int32_t)(fb->pid_sum / (int64_t)fb->pid_count);
    learn_q15 = (int64_t)avg_pid * (int64_t)fb->gain;

    if((fb->learn_accumulator > 0 && learn_q15 < 0) || (fb->learn_accumulator < 0 && learn_q15 > 0))
        app_fb_ff_learning_clear_fraction(fb);

    fb->learn_accumulator += learn_q15;

    if(fb->learn_accumulator >= APP_FB_Q15_ONE || fb->learn_accumulator <= -APP_FB_Q15_ONE)
    {
        int64_t delta = fb->learn_accumulator / APP_FB_Q15_ONE;
        fb->learn_accumulator -= delta * APP_FB_Q15_ONE;
        new_offset = (int64_t)fb->offset + delta;

        if(new_offset > fb->offset_limit) new_offset = fb->offset_limit;
        else if(new_offset < -fb->offset_limit) new_offset = -fb->offset_limit;

        fb->offset = (int32_t)new_offset;

        if((fb->offset >= fb->offset_limit && fb->learn_accumulator > 0) ||
           (fb->offset <= -fb->offset_limit && fb->learn_accumulator < 0))
        {
            app_fb_ff_learning_clear_fraction(fb);
        }
    }

    if(fb->counter != UINT32_MAX) fb->counter++;
    app_fb_ff_learning_clear_window(fb);
    return fb->offset;
}

int32_t app_fb_ff_learning_get_offset(APP_FB_FF_LEARNING_T *fb)
{
    if(fb == 0) return 0;
    return app_fb_ff_learning_limit(fb, fb->offset);
}

#ifdef __cplusplus
}
#endif
