#include "FB_C_integral_separation.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FB_I_HYSTERESIS (5)

void app_fb_integral_separation_init(APP_FB_INTEGRAL_SEPARATION_T *fb, int32_t threshold)
{
    if(fb == 0) return;
    if(threshold < 0) threshold = 0;
    fb->error_threshold = threshold;
    fb->enable = APP_FB_FALSE;
}

APP_FB_BOOL app_fb_integral_separation_run(APP_FB_INTEGRAL_SEPARATION_T *fb, int32_t error)
{
    int32_t abs_error;
    if(fb == 0) return APP_FB_FALSE;
    abs_error = APP_FB_ABS(error);
    if(fb->enable == APP_FB_FALSE)
    {
        if(abs_error <= fb->error_threshold) fb->enable = APP_FB_TRUE;
    }
    else
    {
        if(abs_error >= (fb->error_threshold + APP_FB_I_HYSTERESIS)) fb->enable = APP_FB_FALSE;
    }
    return fb->enable;
}

#ifdef __cplusplus
}
#endif
