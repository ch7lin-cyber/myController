#ifndef SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_
#define SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_

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
    int32_t rise_rate_per_sec;
    int32_t fall_rate_per_sec;
    uint32_t sample_time_ms;

    int64_t rise_remainder;
    int64_t fall_remainder;

    APP_FB_PWM previous;
    APP_FB_PWM output;
} APP_FB_RATE_LIMIT_T;

/* Legacy per-cycle initializer, retained for standalone compatibility. */
void app_fb_rate_limit_init(
    APP_FB_RATE_LIMIT_T *fb,
    int32_t rise_limit,
    int32_t fall_limit);

/* Timing-aware initializer used by the temperature controller. */
APP_FB_ERROR app_fb_rate_limit_init_timed(
    APP_FB_RATE_LIMIT_T *fb,
    uint32_t sample_time_ms,
    int32_t rise_rate_per_sec,
    int32_t fall_rate_per_sec);

void app_fb_rate_limit_reset(APP_FB_RATE_LIMIT_T *fb, int32_t output);
APP_FB_PWM app_fb_rate_limit_run(APP_FB_RATE_LIMIT_T *fb, APP_FB_PWM input);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
