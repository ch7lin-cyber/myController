#include "FB_C_output_rate_limit.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_rate_limit_clamp(int32_t value)
{
    if(value > APP_FB_PWM_MAX) return APP_FB_PWM_MAX;
    if(value < APP_FB_PWM_MIN) return APP_FB_PWM_MIN;
    return value;
}

void app_fb_rate_limit_init(
    APP_FB_RATE_LIMIT_T *fb,
    int32_t rise_limit,
    int32_t fall_limit)
{
    if(fb == 0) return;
    if(rise_limit < 0) rise_limit = 0;
    if(fall_limit < 0) fall_limit = 0;

    /* Legacy compatibility: with Ts=1000 ms, counts/s equals counts/cycle. */
    fb->rise_rate_per_sec = rise_limit;
    fb->fall_rate_per_sec = fall_limit;
    fb->sample_time_ms = 1000U;
    fb->rise_remainder = 0;
    fb->fall_remainder = 0;
    fb->previous = 0;
    fb->output = 0;
}

APP_FB_ERROR app_fb_rate_limit_init_timed(
    APP_FB_RATE_LIMIT_T *fb,
    uint32_t sample_time_ms,
    int32_t rise_rate_per_sec,
    int32_t fall_rate_per_sec)
{
    if(fb == 0) return APP_FB_ERROR_NULL_POINTER;
    if(sample_time_ms < APP_FB_SAMPLE_TIME_MIN_MS ||
       sample_time_ms > APP_FB_SAMPLE_TIME_MAX_MS ||
       rise_rate_per_sec < 0 ||
       fall_rate_per_sec < 0)
    {
        return APP_FB_ERROR_PARAMETER;
    }

    fb->rise_rate_per_sec = rise_rate_per_sec;
    fb->fall_rate_per_sec = fall_rate_per_sec;
    fb->sample_time_ms = sample_time_ms;
    fb->rise_remainder = 0;
    fb->fall_remainder = 0;
    fb->previous = 0;
    fb->output = 0;

    return APP_FB_OK;
}

void app_fb_rate_limit_reset(APP_FB_RATE_LIMIT_T *fb, int32_t output)
{
    if(fb == 0) return;

    output = app_fb_rate_limit_clamp(output);
    fb->previous = output;
    fb->output = output;
    fb->rise_remainder = 0;
    fb->fall_remainder = 0;
}

APP_FB_PWM app_fb_rate_limit_run(APP_FB_RATE_LIMIT_T *fb, APP_FB_PWM input)
{
    int32_t diff;
    int64_t numerator;
    int64_t step;

    if(fb == 0) return 0;

    input = app_fb_rate_limit_clamp(input);
    diff = input - fb->previous;

    if(diff > 0)
    {
        fb->fall_remainder = 0;

        numerator = ((int64_t)fb->rise_rate_per_sec *
                     (int64_t)fb->sample_time_ms) +
                    fb->rise_remainder;
        step = numerator / 1000LL;
        fb->rise_remainder = numerator % 1000LL;

        if(step <= 0)
        {
            fb->output = fb->previous;
        }
        else if((int64_t)diff > step)
        {
            fb->output = app_fb_rate_limit_clamp(
                (int32_t)((int64_t)fb->previous + step));
        }
        else
        {
            fb->output = input;
            fb->rise_remainder = 0;
        }
    }
    else if(diff < 0)
    {
        fb->rise_remainder = 0;

        numerator = ((int64_t)fb->fall_rate_per_sec *
                     (int64_t)fb->sample_time_ms) +
                    fb->fall_remainder;
        step = numerator / 1000LL;
        fb->fall_remainder = numerator % 1000LL;

        if(step <= 0)
        {
            fb->output = fb->previous;
        }
        else if(-(int64_t)diff > step)
        {
            fb->output = app_fb_rate_limit_clamp(
                (int32_t)((int64_t)fb->previous - step));
        }
        else
        {
            fb->output = input;
            fb->fall_remainder = 0;
        }
    }
    else
    {
        fb->output = input;
        fb->rise_remainder = 0;
        fb->fall_remainder = 0;
    }

    fb->output = app_fb_rate_limit_clamp(fb->output);
    fb->previous = fb->output;
    return fb->output;
}

#ifdef __cplusplus
}
#endif
