#ifndef SSM_STD_FB_APP_USER_FEEDFORWARD_TABLE_CODE_H_
#define SSM_STD_FB_APP_USER_FEEDFORWARD_TABLE_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    APP_FB_TEMP temperature;
    APP_FB_PWM pwm;
} APP_FB_FF_POINT_T;

typedef struct
{
    const APP_FB_FF_POINT_T *table;
    int32_t size;
    APP_FB_PWM output;
} APP_FB_FEEDFORWARD_T;

MY_API void app_fb_feedforward_init(APP_FB_FEEDFORWARD_T *fb, const APP_FB_FF_POINT_T *table, int32_t size);
MY_API APP_FB_PWM app_fb_feedforward_run(APP_FB_FEEDFORWARD_T *fb, APP_FB_TEMP sv);

#ifdef __cplusplus
}
#endif
#endif
