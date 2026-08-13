#include <stdint.h>
#include "FB_C_self_tuner.h"
#include "FB_C_adaptive_temperature_controller.h"

/*
 * Runtime predictive-brake horizon used by the existing V3.6 core through
 * APP_FB_PREDICTIVE_BRAKE_TIME_MS. Branch 5 keeps a single controller instance
 * in the current application, so one runtime backing value is sufficient for
 * this integration stage.
 */
uint32_t g_app_fb_predictive_brake_time_ms = 1000U;

static int32_t ci(int32_t v,int32_t a,int32_t b){return v<a?a:(v>b?b:v);}
static uint32_t cu(uint32_t v,uint32_t a,uint32_t b){return v<a?a:(v>b?b:v);}

APP_FB_ERROR app_fb_self_tuner_init(APP_FB_SELF_TUNER_T*f,const APP_FB_SELF_TUNER_PARAMETER_T*p,int32_t ki,uint32_t pt)
{
    if(!f||!p)return APP_FB_ERROR_NULL_POINTER;
    if(p->ki_max<p->ki_min||p->predictive_time_max_ms<p->predictive_time_min_ms)return APP_FB_ERROR_PARAMETER;
    f->param=*p;
    f->enable=APP_FB_TRUE;
    f->update_ready=APP_FB_FALSE;
    f->settled_consumed=APP_FB_FALSE;
    f->suggested_ki=ci(ki,p->ki_min,p->ki_max);
    f->suggested_predictive_time_ms=cu(pt,p->predictive_time_min_ms,p->predictive_time_max_ms);
    return APP_FB_OK;
}

void app_fb_self_tuner_reset(APP_FB_SELF_TUNER_T*f,int32_t ki,uint32_t pt)
{
    if(!f)return;
    f->update_ready=APP_FB_FALSE;
    f->settled_consumed=APP_FB_FALSE;
    f->suggested_ki=ci(ki,f->param.ki_min,f->param.ki_max);
    f->suggested_predictive_time_ms=cu(pt,f->param.predictive_time_min_ms,f->param.predictive_time_max_ms);
}

APP_FB_BOOL app_fb_self_tuner_run(APP_FB_SELF_TUNER_T*f,const APP_FB_PROCESS_METRIC_T*m)
{
    int32_t ki;
    uint32_t pt;

    if(!f||!m||!f->enable)return APP_FB_FALSE;
    f->update_ready=APP_FB_FALSE;

    if(!m->settled)
    {
        f->settled_consumed=APP_FB_FALSE;
        return APP_FB_FALSE;
    }

    if(f->settled_consumed)return APP_FB_FALSE;
    f->settled_consumed=APP_FB_TRUE;

    ki=f->suggested_ki;
    pt=f->suggested_predictive_time_ms;

    if(m->steady_error>f->param.steady_error_threshold)
        ki+=f->param.ki_step;
    else if(m->steady_error<-f->param.steady_error_threshold)
        ki-=f->param.ki_step;

    if(m->overshoot>f->param.overshoot_threshold)
    {
        if(pt<=UINT32_MAX-f->param.predictive_time_step_ms)
            pt+=f->param.predictive_time_step_ms;
    }
    else if(m->overshoot==0&&
            m->steady_error>f->param.steady_error_threshold&&
            pt>f->param.predictive_time_step_ms)
    {
        pt-=f->param.predictive_time_step_ms;
    }

    ki=ci(ki,f->param.ki_min,f->param.ki_max);
    pt=cu(pt,f->param.predictive_time_min_ms,f->param.predictive_time_max_ms);

    if(ki!=f->suggested_ki||pt!=f->suggested_predictive_time_ms)
    {
        f->suggested_ki=ki;
        f->suggested_predictive_time_ms=pt;
        f->update_ready=APP_FB_TRUE;
    }

    return f->update_ready;
}

APP_FB_ERROR app_fb_temperature_controller_self_tuning_init(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_SELF_TUNER_PARAMETER_T *p,
    APP_FB_TEMP band,
    uint32_t settle_ms,
    uint32_t pred_ms)
{
    APP_FB_ERROR s;

    if(!fb||!p)return APP_FB_ERROR_NULL_POINTER;

    s=app_fb_process_observer_init(
        &fb->observer,
        fb->timing.sample_time_ms,
        band,
        settle_ms);
    if(s!=APP_FB_OK)return s;

    s=app_fb_self_tuner_init(
        &fb->self_tuner,
        p,
        fb->pid.param.ki,
        pred_ms);
    if(s!=APP_FB_OK)return s;

    fb->predictive_brake_time_ms=fb->self_tuner.suggested_predictive_time_ms;
    g_app_fb_predictive_brake_time_ms=fb->predictive_brake_time_ms;
    fb->self_tuning_initialized=APP_FB_TRUE;
    return APP_FB_OK;
}

MY_API void app_fb_temperature_controller_run_self_tuning(
    APP_FB_TEMPERATURE_CONTROLLER_T *fb,
    const APP_FB_TEMP_CONTROLLER_INPUT_T *input,
    APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    const APP_FB_PROCESS_METRIC_T *m;
    APP_FB_PID_PARAMETER_T p;

    if(!fb||!input||!output)return;

    /* Existing V3.6 control path executes first. */
    app_fb_temperature_controller_run(fb,input,output);

    if(fb->self_tuning_initialized==APP_FB_FALSE)return;

    if(input->enable==APP_FB_FALSE||input->mode!=APP_FB_MODE_AUTO)
    {
        app_fb_process_observer_reset(&fb->observer);
        app_fb_self_tuner_reset(
            &fb->self_tuner,
            fb->pid.param.ki,
            fb->predictive_brake_time_ms);
        return;
    }

    m=app_fb_process_observer_run(&fb->observer,input->sv,input->pv);
    if(!m)return;

    /* One parameter commit at most for each settled response. */
    if(app_fb_self_tuner_run(&fb->self_tuner,m)==APP_FB_TRUE)
    {
        p=fb->pid.param;
        p.ki=fb->self_tuner.suggested_ki;
        (void)app_fb_pid_reconfigure_timed(&fb->pid,&p);

        fb->predictive_brake_time_ms=
            fb->self_tuner.suggested_predictive_time_ms;
        g_app_fb_predictive_brake_time_ms=
            fb->predictive_brake_time_ms;
    }
}
