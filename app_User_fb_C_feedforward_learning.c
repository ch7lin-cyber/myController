#include "app_User_fb_C_feedforward_learning.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_ff_learning_limit(APP_FB_FF_LEARNING_T *fb, int32_t value)
{
    if(value > fb->offset_limit) return fb->offset_limit;
    if(value < -fb->offset_limit) return -fb->offset_limit;
    return value;
}

static int64_t app_fb_ff_abs_i32(int32_t value)
{
    if(value >= 0) return (int64_t)value;
    return -(int64_t)value;
}

static void app_fb_ff_learning_clear_window(APP_FB_FF_LEARNING_T *fb)
{
    fb->stable_counter = 0;
    fb->pid_sum = 0;
    fb->pid_count = 0;
}

static void app_fb_ff_learning_clear_fraction(APP_FB_FF_LEARNING_T *fb)
{
    fb->learn_accumulator = 0;
}

MY_API void app_fb_ff_learning_init(
    APP_FB_FF_LEARNING_T *fb,
    const APP_FB_ADAPTIVE_PARAMETER_T *param)
{
    if(fb == 0) return;

    fb->error_threshold = APP_FB_ADAPTIVE_ERROR;
    fb->gain = APP_FB_ADAPTIVE_GAIN;
    fb->sv_change_threshold = APP_FB_ADAPTIVE_SV_CHANGE;
    fb->pid_deadband = APP_FB_ADAPTIVE_PID_DEADBAND;
    fb->stable_count = APP_FB_ADAPTIVE_STABLE_COUNT;
    fb->freeze_count = APP_FB_ADAPTIVE_FREEZE_COUNT;
    fb->offset_limit = APP_FB_ADAPTIVE_OFFSET_LIMIT;

    if(param != 0)
    {
        if(param->error_threshold >= 0)
            fb->error_threshold = param->error_threshold;
        if(param->gain >= 0)
            fb->gain = param->gain;
        if(param->sv_change_threshold >= 0)
            fb->sv_change_threshold = param->sv_change_threshold;
        if(param->pid_deadband >= 0)
            fb->pid_deadband = param->pid_deadband;
        if(param->stable_count > 0)
            fb->stable_count = param->stable_count;
        if(param->freeze_count > 0)
            fb->freeze_count = param->freeze_count;
        if(param->offset_limit > 0)
            fb->offset_limit = param->offset_limit;
    }

    fb->counter = 0U;
    fb->offset = 0;
    fb->pid_sum = 0;
    fb->pid_count = 0;
    fb->stable_counter = 0;
    fb->freeze_counter = 0;
    fb->previous_sv = 0;
    fb->sv_initialized = APP_FB_FALSE;
    fb->learn_accumulator = 0;
}

MY_API void app_fb_ff_learning_reset(APP_FB_FF_LEARNING_T *fb)
{
    if(fb == 0) return;

    fb->offset = 0;
    fb->counter = 0U;
    fb->previous_sv = 0;
    fb->sv_initialized = APP_FB_FALSE;
    fb->freeze_counter = 0;
    app_fb_ff_learning_clear_fraction(fb);
    app_fb_ff_learning_clear_window(fb);
}

MY_API int32_t app_fb_ff_learning_run(
    APP_FB_FF_LEARNING_T *fb,
    int32_t sv,
    int32_t pv,
    int32_t pid_output,
    APP_FB_BOOL allow_learning)
{
    int32_t error;
    int32_t avg_pid;
    int64_t learn_q15;
    int64_t new_offset;

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

    if(app_fb_ff_abs_i32(sv - fb->previous_sv) >= fb->sv_change_threshold)
    {
        fb->previous_sv = sv;
        fb->freeze_counter = fb->freeze_count;
        app_fb_ff_learning_clear_fraction(fb);
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    if(fb->freeze_counter > 0)
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

    fb->stable_counter++;
    fb->pid_sum += pid_output;
    fb->pid_count++;

    if(fb->stable_counter < fb->stable_count)
        return fb->offset;

    if(fb->pid_count == 0)
    {
        app_fb_ff_learning_clear_window(fb);
        return fb->offset;
    }

    avg_pid = fb->pid_sum / (int32_t)fb->pid_count;
    learn_q15 = (int64_t)avg_pid * (int64_t)fb->gain;

    if((fb->learn_accumulator > 0 && learn_q15 < 0) ||
       (fb->learn_accumulator < 0 && learn_q15 > 0))
    {
        app_fb_ff_learning_clear_fraction(fb);
    }

    fb->learn_accumulator += learn_q15;

    if(fb->learn_accumulator >= APP_FB_Q15_ONE ||
       fb->learn_accumulator <= -APP_FB_Q15_ONE)
    {
        int64_t delta = fb->learn_accumulator / APP_FB_Q15_ONE;
        fb->learn_accumulator -= delta * APP_FB_Q15_ONE;

        new_offset = (int64_t)fb->offset + delta;
        if(new_offset > fb->offset_limit)
            new_offset = fb->offset_limit;
        else if(new_offset < -fb->offset_limit)
            new_offset = -fb->offset_limit;

        fb->offset = (int32_t)new_offset;

        if((fb->offset >= fb->offset_limit && fb->learn_accumulator > 0) ||
           (fb->offset <= -fb->offset_limit && fb->learn_accumulator < 0))
        {
            app_fb_ff_learning_clear_fraction(fb);
        }
    }

    if(fb->counter != UINT32_MAX)
        fb->counter++;

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
