#ifndef SSM_STD_FB_APP_C_CONTROL_CODE_H_
#define SSM_STD_FB_APP_C_CONTROL_CODE_H_

#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

typedef struct { int32_t integral; int32_t error_previous; int32_t output; int64_t aw_remainder; } APP_FB_PID_STATE_T;
typedef struct { APP_FB_PID_PARAMETER_T param; APP_FB_PID_STATE_T state; APP_FB_BOOL enable; APP_FB_BOOL integral_enable; } APP_FB_PID_T;

void app_fb_pid_init(APP_FB_PID_T *fb, const APP_FB_PID_PARAMETER_T *param);
void app_fb_pid_reset(APP_FB_PID_T *fb);
int32_t app_fb_pid_run(APP_FB_PID_T *fb, APP_FB_TEMP sv, APP_FB_TEMP pv, int32_t d_filtered);
void app_fb_pid_bumpless_preload(APP_FB_PID_T *fb, APP_FB_TEMP sv, APP_FB_TEMP pv, int32_t d_filtered, int32_t desired_pid_output);
void app_fb_pid_anti_windup(APP_FB_PID_T *fb, int32_t unsaturated_output, int32_t actual_output);
void app_fb_pid_integral_add(APP_FB_PID_T *fb, int32_t value);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
