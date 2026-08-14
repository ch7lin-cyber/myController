#ifdef __cplusplus
extern "C" {
#endif

#include "entry_C_adptiveTempController.h"

static const APP_FB_FF_POINT_T heater_ff_table[] =
{
    {.temperature = 500,  .pwm = 2},
    {.temperature = 750,  .pwm = 25},
    {.temperature = 1000, .pwm = 50},
    {.temperature = 1250, .pwm = 200},
    {.temperature = 1500, .pwm = 250},
    {.temperature = 1750, .pwm = 300},
    {.temperature = 2000, .pwm = 320},
    {.temperature = 2500, .pwm = 340},
    {.temperature = 3000, .pwm = 380}
};

static const APP_FB_PID_PARAMETER_T heater_pid =
{
    .kp = 9000,
    .ki = 300,
    .kd = 5000,
    .integral_limit = 32767,
    .output_limit = 900,
    .kaw = APP_FB_PID_KAW_DEFAULT
};

static const APP_FB_SELF_TUNER_PARAMETER_T heater_self_tuner =
{
    .ki_min = APP_FB_SELF_TUNE_KI_MIN,
    .ki_max = APP_FB_SELF_TUNE_KI_MAX,
    .ki_step = APP_FB_SELF_TUNE_KI_STEP,
    .predictive_time_min_ms = APP_FB_SELF_TUNE_PREDICTIVE_TIME_MIN_MS,
    .predictive_time_max_ms = APP_FB_SELF_TUNE_PREDICTIVE_TIME_MAX_MS,
    .predictive_time_step_ms = APP_FB_SELF_TUNE_PREDICTIVE_TIME_STEP_MS,
    .steady_error_threshold = APP_FB_SELF_TUNE_STEADY_ERROR_THRESHOLD,
    .overshoot_threshold = APP_FB_SELF_TUNE_OVERSHOOT_THRESHOLD,
    .min_heating_step = APP_FB_SELF_TUNE_MIN_HEATING_STEP,
    .cooldown_ms = APP_FB_SELF_TUNE_COOLDOWN_MS,
    .max_commits_per_session = APP_FB_SELF_TUNE_MAX_COMMITS_PER_SESSION
};

APP_FB_TEMPERATURE_CONTROLLER_T heater_controller;

MY_API void Heater_Control_Init(void)
{
    (void)Heater_Control_InitExTimed(0, APP_FB_SAMPLE_TIME_DEFAULT_MS);
}

MY_API void Heater_Control_InitEx(const APP_FB_ADAPTIVE_PARAMETER_T *p)
{
    (void)Heater_Control_InitExTimed(p, APP_FB_SAMPLE_TIME_DEFAULT_MS);
}

MY_API APP_FB_ERROR Heater_Control_InitTimed(uint32_t ts)
{
    return Heater_Control_InitExTimed(0, ts);
}

MY_API APP_FB_ERROR Heater_Control_InitExTimed(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter,
    uint32_t sample_time_ms)
{
    APP_FB_TIMING_PARAMETER_T timing;
    APP_FB_ERROR s;

    timing.sample_time_ms = sample_time_ms;

    s = app_fb_temperature_controller_init_ex_timed(
        &heater_controller,
        heater_ff_table,
        (int32_t)(sizeof(heater_ff_table) / sizeof(APP_FB_FF_POINT_T)),
        &heater_pid,
        adaptive_parameter,
        &timing);
    if(s != APP_FB_OK) return s;

    /* Initializes diagnostics and tuner state; tuner itself remains OFF. */
    return app_fb_temperature_controller_self_tuning_init(
        &heater_controller,
        &heater_self_tuner,
        APP_FB_SELF_TUNE_SETTLE_BAND,
        APP_FB_SELF_TUNE_SETTLE_REQUIRED_MS,
        APP_FB_PREDICTIVE_BRAKE_TIME_DEFAULT_MS);
}

MY_API uint32_t Heater_GetSampleTimeMs(void)
{
    return app_fb_temperature_controller_get_sample_time_ms(&heater_controller);
}

MY_API void Heater_SetAdaptiveParameter(const APP_FB_ADAPTIVE_PARAMETER_T *p)
{
    app_fb_temperature_controller_set_adaptive_parameter(&heater_controller, p);
}

MY_API void Heater_SetSelfTuningEnable(APP_FB_BOOL enable)
{
    app_fb_self_tuner_set_enable(&heater_controller.self_tuner, enable);
}

MY_API APP_FB_BOOL Heater_GetSelfTuningEnable(void)
{
    return heater_controller.self_tuner.enable;
}

MY_API APP_FB_ERROR Heater_GetSelfTuningDiagnostics(
    HEATER_SELF_TUNING_DIAGNOSTICS_T *d)
{
    const APP_FB_PROCESS_METRIC_T *m;

    if(d == 0) return APP_FB_ERROR_NULL_POINTER;
    if(heater_controller.self_tuning_initialized == APP_FB_FALSE)
        return APP_FB_ERROR_PARAMETER;

    m = &heater_controller.observer.metric;

    d->self_tuning_enabled = heater_controller.self_tuner.enable;
    d->response_active = m->response_active;
    d->settled = m->settled;
    d->current_ki = heater_controller.pid.param.ki;
    d->current_predictive_time_ms = heater_controller.predictive_brake_time_ms;
    d->peak_pv = m->peak_pv;
    d->overshoot = m->overshoot;
    d->steady_error = m->steady_error;
    d->pv_slope_per_s = m->pv_slope_per_s;
    d->elapsed_time_ms = m->elapsed_time_ms;
    d->settling_time_ms = m->settling_time_ms;
    d->tune_count = heater_controller.self_tuner.tune_count;
    d->last_tune_reason = heater_controller.self_tuner.last_tune_reason;

    return APP_FB_OK;
}

MY_API void Heater_myAdptiveControl(
    int16_t input_pv,
    int16_t input_sv,
    int32_t *output_pid_out,
    int32_t *output_ff_pwm,
    int32_t *output_ff_offset,
    int32_t *output_heater_pwm)
{
    static APP_FB_TEMP_CONTROLLER_INPUT_T input;
    static APP_FB_TEMP_CONTROLLER_OUTPUT_T output;

    if(!output_pid_out || !output_ff_pwm || !output_ff_offset || !output_heater_pwm)
        return;

    input.enable = APP_FB_TRUE;
    input.sv = input_sv;
    input.pv = input_pv;
    input.mode = APP_FB_MODE_AUTO;

    /* Diagnostics run every AUTO cycle; parameter adaptation is opt-in. */
    app_fb_temperature_controller_run_self_tuning(
        &heater_controller,
        &input,
        &output);

    *output_pid_out = output.pid_output;
    *output_ff_pwm = output.ff_pwm;
    *output_ff_offset = output.ff_offset;
    *output_heater_pwm = output.pwm;
}

#ifdef __cplusplus
}
#endif
