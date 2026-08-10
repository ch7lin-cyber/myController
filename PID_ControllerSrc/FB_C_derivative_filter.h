#ifndef SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_
#define SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef struct
{
    int32_t alpha;
    uint32_t sample_time_ms;
    uint32_t time_constant_ms;

    /* Filter state is retained in Q15 precision to prevent integer-limit-cycle
     * residuals when alpha is close to 1 at fast sample times. */
    int64_t output_q15;
    int32_t output;
    int32_t pv_previous;
    APP_FB_BOOL initialized;
} APP_FB_D_FILTER_T;

/* Legacy alpha-based init retained for compatibility. */
void app_fb_d_filter_init(APP_FB_D_FILTER_T *fb, int32_t alpha);

/* Preferred timing-aware init. Alpha is calculated once during init as:
 * alpha = tau / (tau + Ts), represented in Q15. */
APP_FB_ERROR app_fb_d_filter_init_timed(
    APP_FB_D_FILTER_T *fb,
    uint32_t sample_time_ms,
    uint32_t time_constant_ms);

void app_fb_d_filter_reset(APP_FB_D_FILTER_T *fb);
int32_t app_fb_d_filter_run(APP_FB_D_FILTER_T *fb, int32_t pv);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
