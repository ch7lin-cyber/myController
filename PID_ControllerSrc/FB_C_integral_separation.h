#ifndef SSM_STD_FB_APP_USER_INTEGRAL_SEPARATION_CODE_H_
#define SSM_STD_FB_APP_USER_INTEGRAL_SEPARATION_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t error_threshold;
    APP_FB_BOOL enable;
} APP_FB_INTEGRAL_SEPARATION_T;

void app_fb_integral_separation_init(APP_FB_INTEGRAL_SEPARATION_T *fb, int32_t threshold);
APP_FB_BOOL app_fb_integral_separation_run(APP_FB_INTEGRAL_SEPARATION_T *fb, int32_t error);

#ifdef __cplusplus
}
#endif
#endif
