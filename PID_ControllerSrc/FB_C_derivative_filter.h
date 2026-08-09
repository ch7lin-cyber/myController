#ifndef SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_
#define SSM_STD_FB_APP_USER_DERIVATIVE_FILTER_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t alpha;
    int32_t output;
    int32_t pv_previous;
    APP_FB_BOOL initialized;
} APP_FB_D_FILTER_T;

void app_fb_d_filter_init(APP_FB_D_FILTER_T *fb, int32_t alpha);
void app_fb_d_filter_reset(APP_FB_D_FILTER_T *fb);
int32_t app_fb_d_filter_run(APP_FB_D_FILTER_T *fb, int32_t pv);

#ifdef __cplusplus
}
#endif
#endif
