#include "FB_C_feedforward_table.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_ff_interpolation(int32_t x, int32_t x1, int32_t x2, int32_t y1, int32_t y2)
{
    int64_t result;
    if(x2 == x1) return y1;
    result = (int64_t)(x-x1) * (y2-y1);
    result /= (x2-x1);
    result += y1;
    return (int32_t)result;
}

MY_API void app_fb_feedforward_init(APP_FB_FEEDFORWARD_T *fb, const APP_FB_FF_POINT_T *table, int32_t size)
{
    if(fb == 0) return;
    fb->table = table;
    fb->size = size;
    fb->output = 0;
}

MY_API APP_FB_PWM app_fb_feedforward_run(APP_FB_FEEDFORWARD_T *fb, APP_FB_TEMP sv)
{
    int32_t i;
    int32_t pwm;
    if(fb == 0 || fb->table == 0 || fb->size <= 0) return 0;
    if(sv <= fb->table[0].temperature)
    {
        fb->output = fb->table[0].pwm;
        return fb->output;
    }
    if(sv >= fb->table[fb->size-1].temperature)
    {
        fb->output = fb->table[fb->size-1].pwm;
        return fb->output;
    }
    for(i=0;i<(fb->size-1);i++)
    {
        if((sv >= fb->table[i].temperature) && (sv <= fb->table[i+1].temperature))
        {
            pwm = app_fb_ff_interpolation(sv, fb->table[i].temperature, fb->table[i+1].temperature, fb->table[i].pwm, fb->table[i+1].pwm);
            fb->output = APP_FB_LIMIT(pwm, APP_FB_PWM_MIN, APP_FB_PWM_MAX);
            return fb->output;
        }
    }
    fb->output = 0;
    return fb->output;
}

#ifdef __cplusplus
}
#endif
