#include <stdint.h>
#include "FB_C_self_tuner.h"

static int32_t limit_i32(int32_t v, int32_t lo, int32_t hi){ if(v<lo)return lo; if(v>hi)return hi; return v; }
static uint32_t limit_u32(uint32_t v,uint32_t lo,uint32_t hi){ if(v<lo)return lo; if(v>hi)return hi; return v; }

APP_FB_ERROR app_fb_self_tuner_init(APP_FB_SELF_TUNER_T *fb,const APP_FB_SELF_TUNER_PARAMETER_T *p,int32_t ki,uint32_t pt)
{
    if(!fb||!p) return APP_FB_ERROR_NULL_POINTER;
    if(p->ki_min<0||p->ki_max<p->ki_min||p->ki_step<0||p->predictive_time_min_ms==0U||p->predictive_time_max_ms<p->predictive_time_min_ms||p->predictive_time_step_ms==0U)
        return APP_FB_ERROR_PARAMETER;
    fb->param=*p; fb->enable=APP_FB_TRUE; fb->update_ready=APP_FB_FALSE;
    fb->suggested_ki=limit_i32(ki,p->ki_min,p->ki_max);
    fb->suggested_predictive_time_ms=limit_u32(pt,p->predictive_time_min_ms,p->predictive_time_max_ms);
    return APP_FB_OK;
}

void app_fb_self_tuner_reset(APP_FB_SELF_TUNER_T *fb,int32_t ki,uint32_t pt)
{
    if(!fb) return;
    fb->update_ready=APP_FB_FALSE;
    fb->suggested_ki=limit_i32(ki,fb->param.ki_min,fb->param.ki_max);
    fb->suggested_predictive_time_ms=limit_u32(pt,fb->param.predictive_time_min_ms,fb->param.predictive_time_max_ms);
}

APP_FB_BOOL app_fb_self_tuner_run(APP_FB_SELF_TUNER_T *fb,const APP_FB_PROCESS_METRIC_T *m)
{
    int32_t ki; uint32_t pt;
    if(!fb||!m) return APP_FB_FALSE;
    fb->update_ready=APP_FB_FALSE;
    if(!fb->enable||!m->settled) return APP_FB_FALSE;
    ki=fb->suggested_ki; pt=fb->suggested_predictive_time_ms;
    if(m->steady_error>fb->param.steady_error_threshold) ki+=fb->param.ki_step;
    else if(m->steady_error<-fb->param.steady_error_threshold) ki-=fb->param.ki_step;
    if(m->overshoot>fb->param.overshoot_threshold)
    {
        if(pt<=UINT32_MAX-fb->param.predictive_time_step_ms) pt+=fb->param.predictive_time_step_ms;
    }
    else if(m->overshoot==0&&m->steady_error>fb->param.steady_error_threshold)
    {
        if(pt>fb->param.predictive_time_step_ms) pt-=fb->param.predictive_time_step_ms;
    }
    ki=limit_i32(ki,fb->param.ki_min,fb->param.ki_max);
    pt=limit_u32(pt,fb->param.predictive_time_min_ms,fb->param.predictive_time_max_ms);
    if(ki!=fb->suggested_ki||pt!=fb->suggested_predictive_time_ms)
    { fb->suggested_ki=ki; fb->suggested_predictive_time_ms=pt; fb->update_ready=APP_FB_TRUE; }
    return fb->update_ready;
}
