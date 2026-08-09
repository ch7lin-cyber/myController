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

MY_API void app_fb_rate_limit_init(APP_FB_RATE_LIMIT_T *fb, int32_t rise_limit, int32_t fall_limit)
{
    if(fb == 0) return;
    if(rise_limit < 0) rise_limit = 0;
    if(fall_limit < 0) fall_limit = 0;
    fb->rise_limit = rise_limit;
    fb->fall_limit = fall_limit;
    fb->previous = 0;
    fb->output = 0;
}

MY_API void app_fb_rate_limit_reset(APP_FB_RATE_LIMIT_T *fb, int32_t output)
{
    if(fb == 0) return;
    output = app_fb_rate_limit_clamp(output);
    fb->previous = output;
    fb->output = output;
}

MY_API APP_FB_PWM app_fb_rate_limit_run(APP_FB_RATE_LIMIT_T *fb, APP_FB_PWM input)
{
    int32_t diff;
    if(fb == 0) return 0;
    input = app_fb_rate_limit_clamp(input);
    diff = input - fb->previous;
    if(diff > fb->rise_limit) fb->output = fb->previous + fb->rise_limit;
    else if(diff < -fb->fall_limit) fb->output = fb->previous - fb->fall_limit;
    else fb->output = input;
    fb->output = app_fb_rate_limit_clamp(fb->output);
    fb->previous = fb->output;
    return fb->output;
}

#ifdef __cplusplus
}
#endif
