#ifndef SSM_STD_FB_APP_C_CONTROL_CODE_H_
#define SSM_STD_FB_APP_C_CONTROL_CODE_H_
#include <stdint.h>
#include "FB_C_control_type.h"
#include "FB_C_parameter.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct{int32_t integral;int32_t error_previous;int32_t output;int64_t aw_remainder;} APP_FB_PID_STATE_T;
typedef struct{APP_FB_PID_PARAMETER_T param;APP_FB_PID_PARAMETER_T runtime_param;APP_FB_PID_STATE_T state;uint32_t sample_time_ms;int32_t aw_max_correction_runtime;APP_FB_BOOL enable;APP_FB_BOOL integral_enable;} APP_FB_PID_T;

void app_fb_pid_init(APP_FB_PID_T *fb,const APP_FB_PID_PARAMETER_T *param);
APP_FB_ERROR app_fb_pid_init_timed(APP_FB_PID_T *fb,const APP_FB_PID_PARAMETER_T *param,uint32_t sample_time_ms);

static inline int32_t app_fb_pid_scale_reconfig(int32_t v,uint32_t n,uint32_t d)
{
    int64_t p;
    if(d==0U)return 0;
    p=(int64_t)v*(int64_t)n;
    return (int32_t)(p/(int64_t)d);
}

/*
 * Reconfigure reference gains without resetting PID state.
 * When Ki changes, integral state is inversely rescaled so the I-term remains
 * approximately bumpless: Ki_old * I_old ~= Ki_new * I_new.
 */
static inline APP_FB_ERROR app_fb_pid_reconfigure_timed(
    APP_FB_PID_T *fb,
    const APP_FB_PID_PARAMETER_T *param)
{
    int32_t il;
    int32_t old_runtime_ki;
    int32_t old_integral;
    int64_t integral_scaled;

    if(!fb||!param)return APP_FB_ERROR_NULL_POINTER;
    if(fb->sample_time_ms<APP_FB_SAMPLE_TIME_MIN_MS||fb->sample_time_ms>APP_FB_SAMPLE_TIME_MAX_MS)
        return APP_FB_ERROR_PARAMETER;

    old_runtime_ki=fb->runtime_param.ki;
    old_integral=fb->state.integral;

    fb->param=*param;
    fb->runtime_param=*param;
    fb->runtime_param.kp=param->kp;
    fb->runtime_param.ki=app_fb_pid_scale_reconfig(
        param->ki,
        fb->sample_time_ms,
        APP_FB_PID_REFERENCE_SAMPLE_TIME_MS);
    fb->runtime_param.kd=app_fb_pid_scale_reconfig(
        param->kd,
        APP_FB_PID_REFERENCE_SAMPLE_TIME_MS,
        fb->sample_time_ms);

    il=app_fb_pid_scale_reconfig(
        param->integral_limit,
        APP_FB_PID_REFERENCE_SAMPLE_TIME_MS,
        fb->sample_time_ms);
    if(param->integral_limit>0&&il<1)il=1;
    if(il<0)il=0;
    fb->runtime_param.integral_limit=il;
    fb->runtime_param.output_limit=param->output_limit;
    fb->runtime_param.kaw=app_fb_pid_scale_reconfig(
        param->kaw,
        fb->sample_time_ms,
        APP_FB_PID_REFERENCE_SAMPLE_TIME_MS);

    if(old_runtime_ki!=0&&fb->runtime_param.ki!=0&&old_runtime_ki!=fb->runtime_param.ki)
    {
        integral_scaled=((int64_t)old_integral*(int64_t)old_runtime_ki)/(int64_t)fb->runtime_param.ki;
        if(integral_scaled>(int64_t)il)integral_scaled=(int64_t)il;
        else if(integral_scaled<-(int64_t)il)integral_scaled=-(int64_t)il;
        fb->state.integral=(int32_t)integral_scaled;
    }
    else
    {
        if(fb->state.integral>il)fb->state.integral=il;
        else if(fb->state.integral<-il)fb->state.integral=-il;
    }

    fb->state.aw_remainder=0;
    return APP_FB_OK;
}

void app_fb_pid_reset(APP_FB_PID_T *fb);
int32_t app_fb_pid_run(APP_FB_PID_T *fb,APP_FB_TEMP sv,APP_FB_TEMP pv,int32_t d_filtered);
void app_fb_pid_bumpless_preload(APP_FB_PID_T *fb,APP_FB_TEMP sv,APP_FB_TEMP pv,int32_t d_filtered,int32_t desired_pid_output);
void app_fb_pid_anti_windup(APP_FB_PID_T *fb,int32_t unsaturated_output,int32_t actual_output);
void app_fb_pid_integral_add(APP_FB_PID_T *fb,int32_t value);

#ifdef __cplusplus
}
#endif
#endif
