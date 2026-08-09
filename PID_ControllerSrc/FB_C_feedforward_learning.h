#ifndef SSM_STD_FB_APP_USER_FF_LEARNING_CODE_H_
#define SSM_STD_FB_APP_USER_FF_LEARNING_CODE_H_

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
    int32_t error_threshold;
    int32_t gain;
    int32_t sv_change_threshold;
    int32_t pid_deadband;
    uint16_t stable_count;
    uint16_t freeze_count;
    int32_t offset_limit;
    uint32_t counter;
    int32_t offset;
    int32_t pid_sum;
    uint16_t pid_count;
    uint16_t stable_counter;
    uint16_t freeze_counter;
    int32_t previous_sv;
    APP_FB_BOOL sv_initialized;
    int64_t learn_accumulator;
} APP_FB_FF_LEARNING_T;

void app_fb_ff_learning_init(APP_FB_FF_LEARNING_T *fb, const APP_FB_ADAPTIVE_PARAMETER_T *param);
void app_fb_ff_learning_reset(APP_FB_FF_LEARNING_T *fb);
int32_t app_fb_ff_learning_run(APP_FB_FF_LEARNING_T *fb, int32_t sv, int32_t pv, int32_t pid_output, APP_FB_BOOL allow_learning);
int32_t app_fb_ff_learning_get_offset(APP_FB_FF_LEARNING_T *fb);

#define APP_FB_FF_ERROR_DEADBAND  APP_FB_ADAPTIVE_ERROR_DEADBAND
#define APP_FB_FF_PID_DEADBAND    APP_FB_ADAPTIVE_PID_DEADBAND
#define APP_FB_FF_STABLE_COUNT    APP_FB_ADAPTIVE_STABLE_COUNT
#define APP_FB_FF_FREEZE_COUNT    APP_FB_ADAPTIVE_FREEZE_COUNT
#define APP_FB_FF_OFFSET_LIMIT    APP_FB_ADAPTIVE_OFFSET_LIMIT
#define APP_FB_FF_SV_CHANGE       APP_FB_ADAPTIVE_SV_CHANGE

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif
