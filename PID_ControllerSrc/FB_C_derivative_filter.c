#include "FB_C_derivative_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

static int32_t app_fb_d_filter_q15_to_i32_round(int64_t value_q15)
{
    if(value_q15 >= 0)
        return (int32_t)((value_q15 + (APP_FB_Q15_ONE / 2)) / APP_FB_Q15_ONE);

    return (int32_t)((value_q15 - (APP_FB_Q15_ONE / 2)) / APP_FB_Q15_ONE);
}

void app_fb_d_filter_init(APP_FB_D_FILTER_T *fb, int32_t alpha)
{
    if(fb == 0) return;
    if(alpha < 0) alpha = 0;
    if(alpha > 32767) alpha = 32767;

    fb->alpha = alpha;
    fb->sample_time_ms = APP_FB_PID_REFERENCE_SAMPLE_TIME_MS;
    fb->time_constant_ms = 0U;
    fb->output_q15 = 0;
    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
}

APP_FB_ERROR app_fb_d_filter_init_timed(
    APP_FB_D_FILTER_T *fb,
    uint32_t sample_time_ms,
    uint32_t time_constant_ms)
{
    uint64_t numerator;
    uint64_t denominator;
    uint64_t alpha_q15;

    if(fb == 0) return APP_FB_ERROR_NULL_POINTER;
    if(sample_time_ms < APP_FB_SAMPLE_TIME_MIN_MS ||
       sample_time_ms > APP_FB_SAMPLE_TIME_MAX_MS)
    {
        return APP_FB_ERROR_PARAMETER;
    }

    fb->sample_time_ms = sample_time_ms;
    fb->time_constant_ms = time_constant_ms;

    if(time_constant_ms == 0U)
    {
        fb->alpha = 0;
    }
    else
    {
        numerator = (uint64_t)time_constant_ms * (uint64_t)APP_FB_Q15_ONE;
        denominator = (uint64_t)time_constant_ms + (uint64_t)sample_time_ms;
        alpha_q15 = numerator / denominator;

        if(alpha_q15 > 32767ULL) alpha_q15 = 32767ULL;
        fb->alpha = (int32_t)alpha_q15;
    }

    fb->output_q15 = 0;
    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
    return APP_FB_OK;
}

void app_fb_d_filter_reset(APP_FB_D_FILTER_T *fb)
{
    if(fb == 0) return;
    fb->output_q15 = 0;
    fb->output = 0;
    fb->pv_previous = 0;
    fb->initialized = APP_FB_FALSE;
}

int32_t app_fb_d_filter_run(APP_FB_D_FILTER_T *fb, int32_t pv)
{
    int32_t d_pv;
    int64_t state_term_q15;
    int64_t input_term_q15;

    if(fb == 0) return 0;

    if(fb->initialized == APP_FB_FALSE)
    {
        fb->pv_previous = pv;
        fb->output_q15 = 0;
        fb->output = 0;
        fb->initialized = APP_FB_TRUE;
        return 0;
    }

    d_pv = pv - fb->pv_previous;
    fb->pv_previous = pv;

    /*
     * Preserve the recursive state in Q15 instead of truncating it to an
     * integer derivative every cycle. This removes the non-zero residual
     * limit cycle that appears when alpha is close to 1 (for example Ts=1ms).
     */
    state_term_q15 = ((int64_t)fb->alpha * fb->output_q15) >> 15;
    input_term_q15 = (int64_t)(APP_FB_Q15_ONE - fb->alpha) * (int64_t)d_pv;
    fb->output_q15 = state_term_q15 + input_term_q15;

    fb->output = app_fb_d_filter_q15_to_i32_round(fb->output_q15);
    return fb->output;
}

#ifdef __cplusplus
}
#endif
