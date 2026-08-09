#include "FB_C_derivative_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_fb_d_filter_init(APP_FB_D_FILTER_T *fb, int32_t alpha)
{
    if(fb == 0) return;
    if(alpha < 0) alpha = 0;
    if(alpha > 32767) alpha = 32767;
    fb->alpha = alpha;
    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
}

void app_fb_d_filter_reset(APP_FB_D_FILTER_T *fb)
{
    if(fb == 0) return;
    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
}

MY_API int32_t app_fb_d_filter_run(APP_FB_D_FILTER_T *fb, int32_t pv)
{
    int32_t d_pv;
    int64_t result;
    if(fb == 0) return 0;
    if(fb->initialized == APP_FB_FALSE)
    {
        fb->pv_previous = pv;
        fb->output = 0;
        fb->initialized = APP_FB_TRUE;
        return 0;
    }
    d_pv = pv - fb->pv_previous;
    fb->pv_previous = pv;
    result = ((int64_t)fb->alpha * fb->output) >> 15;
    result += ((int64_t)(APP_FB_Q15_ONE - fb->alpha) * d_pv) >> 15;
    fb->output = (int32_t)result;
    return fb->output;
}

#ifdef __cplusplus
}
#endif
