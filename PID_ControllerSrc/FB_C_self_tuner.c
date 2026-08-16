#include <stdint.h>
#include "FB_C_self_tuner.h"
#include "FB_C_adaptive_temperature_controller.h"

uint32_t g_app_fb_predictive_brake_time_ms = 1000U;

static int32_t ci(int32_t v,int32_t a,int32_t b){return v<a?a:(v>b?b:v);}
static uint32_t cu(uint32_t v,uint32_t a,uint32_t b){return v<a?a:(v>b?b:v);}

APP_FB_ERROR app_fb_self_tuner_init(APP_FB_SELF_TUNER_T*f,const APP_FB_SELF_TUNER_PARAMETER_T*p,int32_t ki,uint32_t pt)
{
    if(!f||!p)return APP_FB_ERROR_NULL_POINTER;
    if(p->ki_max<p->ki_min||p->predictive_time_max_ms<p->predictive_time_min_ms||p->min_heating_step<0)return APP_FB_ERROR_PARAMETER;
    f->param=*p;
    f->enable=APP_FB_FALSE;
    f->update_ready=APP_FB_FALSE;
    f->settled_consumed=APP_FB_TRUE;
    f->response_qualified=APP_FB_FALSE;
    f->response_start_sv=0;
    f->response_start_pv=0;
    f->cooldown_remaining_ms=0U;
    f->suggested_ki=ci(ki,p->ki_min,p->ki_max);
    f->suggested_predictive_time_ms=cu(pt,p->predictive_time_min_ms,p->predictive_time_max_ms);
    f->tune_count=0U;
    f->last_tune_reason=APP_FB_SELF_TUNE_REASON_NONE;
    return APP_FB_OK;
}

void app_fb_self_tuner_reset(APP_FB_SELF_TUNER_T*f,int32_t ki,uint32_t pt)
{
    if(!f)return;
    f->update_ready=APP_FB_FALSE;
    f->settled_consumed=APP_FB_TRUE;
    f->response_qualified=APP_FB_FALSE;
    f->response_start_sv=0;
    f->response_start_pv=0;
    f->suggested_ki=ci(ki,f->param.ki_min,f->param.ki_max);
    f->suggested_predictive_time_ms=cu(pt,f->param.predictive_time_min_ms,f->param.predictive_time_max_ms);
}

void app_fb_self_tuner_set_enable(APP_FB_SELF_TUNER_T *f,APP_FB_BOOL enable)
{
    if(!f)return;
    f->enable=(enable!=APP_FB_FALSE)?APP_FB_TRUE:APP_FB_FALSE;
    f->update_ready=APP_FB_FALSE;
    f->settled_consumed=APP_FB_TRUE;
    f->response_qualified=APP_FB_FALSE;
}

void app_fb_self_tuner_begin_response(APP_FB_SELF_TUNER_T *f,APP_FB_TEMP sv,APP_FB_TEMP pv,APP_FB_BOOL qualified)
{
    if(!f)return;
    f->response_start_sv=sv;
    f->response_start_pv=pv;
    f->response_qualified=(qualified!=APP_FB_FALSE)?APP_FB_TRUE:APP_FB_FALSE;
    f->settled_consumed=APP_FB_FALSE;
    f->update_ready=APP_FB_FALSE;
}

void app_fb_self_tuner_tick(APP_FB_SELF_TUNER_T *f,uint32_t elapsed_ms)
{
    if(!f||f->cooldown_remaining_ms==0U)return;
    if(elapsed_ms>=f->cooldown_remaining_ms)f->cooldown_remaining_ms=0U;
    else f->cooldown_remaining_ms-=elapsed_ms;
}

APP_FB_BOOL app_fb_self_tuner_run(APP_FB_SELF_TUNER_T*f,const APP_FB_PROCESS_METRIC_T*m)
{
    int32_t ki;
    uint32_t pt;
    APP_FB_BOOL ki_changed=APP_FB_FALSE;
    APP_FB_BOOL pt_changed=APP_FB_FALSE;
    APP_FB_SELF_TUNE_REASON_T ki_reason=APP_FB_SELF_TUNE_REASON_NONE;
    APP_FB_SELF_TUNE_REASON_T pt_reason=APP_FB_SELF_TUNE_REASON_NONE;

    if(!f||!m||!f->enable)return APP_FB_FALSE;
    f->update_ready=APP_FB_FALSE;
    if(!m->settled)return APP_FB_FALSE;
    if(f->settled_consumed)return APP_FB_FALSE;
    f->settled_consumed=APP_FB_TRUE;

    if(f->response_qualified==APP_FB_FALSE)return APP_FB_FALSE;
    if(f->cooldown_remaining_ms!=0U)return APP_FB_FALSE;
    if(f->param.max_commits_per_session!=0U&&f->tune_count>=f->param.max_commits_per_session)return APP_FB_FALSE;

    ki=f->suggested_ki;
    pt=f->suggested_predictive_time_ms;

    if(m->steady_error>f->param.steady_error_threshold)
    {
        ki+=f->param.ki_step;
        ki_reason=APP_FB_SELF_TUNE_REASON_KI_INCREASE;
    }
    else if(m->steady_error<-f->param.steady_error_threshold)
    {
        ki-=f->param.ki_step;
        ki_reason=APP_FB_SELF_TUNE_REASON_KI_DECREASE;
    }

    if(m->overshoot>f->param.overshoot_threshold)
    {
        if(pt<=UINT32_MAX-f->param.predictive_time_step_ms)
        {
            pt+=f->param.predictive_time_step_ms;
            pt_reason=APP_FB_SELF_TUNE_REASON_PREDICTIVE_TIME_INCREASE;
        }
    }
    else if(m->overshoot==0&&m->steady_error>f->param.steady_error_threshold&&pt>f->param.predictive_time_step_ms)
    {
        pt-=f->param.predictive_time_step_ms;
        pt_reason=APP_FB_SELF_TUNE_REASON_PREDICTIVE_TIME_DECREASE;
    }

    ki=ci(ki,f->param.ki_min,f->param.ki_max);
    pt=cu(pt,f->param.predictive_time_min_ms,f->param.predictive_time_max_ms);
    ki_changed=(ki!=f->suggested_ki)?APP_FB_TRUE:APP_FB_FALSE;
    pt_changed=(pt!=f->suggested_predictive_time_ms)?APP_FB_TRUE:APP_FB_FALSE;

    if(ki_changed||pt_changed)
    {
        f->suggested_ki=ki;
        f->suggested_predictive_time_ms=pt;
        f->update_ready=APP_FB_TRUE;
        if(f->tune_count<UINT32_MAX)f->tune_count++;
        f->cooldown_remaining_ms=f->param.cooldown_ms;
        if(ki_changed&&pt_changed)f->last_tune_reason=APP_FB_SELF_TUNE_REASON_MULTIPLE;
        else if(ki_changed)f->last_tune_reason=ki_reason;
        else f->last_tune_reason=pt_reason;
    }
    return f->update_ready;
}

APP_FB_ERROR app_fb_temperature_controller_self_tuning_init(APP_FB_TEMPERATURE_CONTROLLER_T *fb,const APP_FB_SELF_TUNER_PARAMETER_T *p,APP_FB_TEMP band,uint32_t settle_ms,uint32_t pred_ms)
{
    APP_FB_ERROR s;
    if(!fb||!p)return APP_FB_ERROR_NULL_POINTER;
    s=app_fb_process_observer_init(&fb->observer,fb->timing.sample_time_ms,band,settle_ms);if(s!=APP_FB_OK)return s;
    s=app_fb_self_tuner_init(&fb->self_tuner,p,fb->pid.param.ki,pred_ms);if(s!=APP_FB_OK)return s;
    fb->predictive_brake_time_ms=fb->self_tuner.suggested_predictive_time_ms;
    g_app_fb_predictive_brake_time_ms=fb->predictive_brake_time_ms;
    fb->self_tuning_initialized=APP_FB_TRUE;
    return APP_FB_OK;
}

MY_API void app_fb_temperature_controller_run_self_tuning(APP_FB_TEMPERATURE_CONTROLLER_T *fb,const APP_FB_TEMP_CONTROLLER_INPUT_T *input,APP_FB_TEMP_CONTROLLER_OUTPUT_T *output)
{
    const APP_FB_PROCESS_METRIC_T *m;
    APP_FB_PID_PARAMETER_T p;
    APP_FB_BOOL new_response;
    APP_FB_BOOL qualified;
    int32_t step_value;

    if(!fb||!input||!output)return;
    app_fb_temperature_controller_run(fb,input,output);
    if(fb->self_tuning_initialized==APP_FB_FALSE)return;

    app_fb_self_tuner_tick(&fb->self_tuner,fb->timing.sample_time_ms);

    if(input->enable==APP_FB_FALSE||input->mode!=APP_FB_MODE_AUTO)
    {
        app_fb_process_observer_reset(&fb->observer);
        app_fb_self_tuner_reset(&fb->self_tuner,fb->pid.param.ki,fb->predictive_brake_time_ms);
        return;
    }

    new_response=(fb->observer.initialized==APP_FB_FALSE||input->sv!=fb->observer.active_sv)?APP_FB_TRUE:APP_FB_FALSE;
    if(new_response==APP_FB_TRUE)
    {
        if(fb->observer.initialized==APP_FB_FALSE)
        {
            /* First AUTO response: qualify only when meaningful heating demand exists. */
            step_value=(int32_t)input->sv-(int32_t)input->pv;
        }
        else
        {
            /* Later responses: require the commanded SV itself to rise enough. */
            step_value=(int32_t)input->sv-(int32_t)fb->observer.active_sv;
        }

        qualified=(step_value>=(int32_t)fb->self_tuner.param.min_heating_step)?APP_FB_TRUE:APP_FB_FALSE;
        app_fb_self_tuner_begin_response(&fb->self_tuner,input->sv,input->pv,qualified);
    }

    m=app_fb_process_observer_run(&fb->observer,input->sv,input->pv);
    if(!m)return;

    if(app_fb_self_tuner_run(&fb->self_tuner,m)==APP_FB_TRUE)
    {
        p=fb->pid.param;
        p.ki=fb->self_tuner.suggested_ki;
        (void)app_fb_pid_reconfigure_timed(&fb->pid,&p);
        fb->predictive_brake_time_ms=fb->self_tuner.suggested_predictive_time_ms;
        g_app_fb_predictive_brake_time_ms=fb->predictive_brake_time_ms;
    }
}
