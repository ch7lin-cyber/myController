#ifndef SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_
#define SSM_STD_FB_APP_USER_RATE_LIMIT_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t rise_limit;
    int32_t fall_limit;
    APP_FB_PWM previous;
    APP_FB_PWM output;
} APP_FB_RATE_LIMIT_T;

void app_fb_rate_limit_init(APP_FB_RATE_LIMIT_T *fb, int32_t rise_limit, int32_t fall_limit);
void app_fb_rate_limit_reset(APP_FB_RATE_LIMIT_T *fb, int32_t output);
APP_FB_PWM app_fb_rate_limit_run(APP_FB_RATE_LIMIT_T *fb, APP_FB_PWM input);

#ifdef __cplusplus
}
#endif
#endif
