/***************************************************************
Description :
    Application wrapper for the adaptive temperature controller.

Timing policy:
    The outer scheduler/application owns the execution period.
    sample_time_ms is supplied only during controller initialization and must
    remain fixed while the controller is running.
***************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "entry_C_adptiveTempController.h"

static const APP_FB_FF_POINT_T heater_ff_table[] =
{
    {.temperature = 500, .pwm = 2},
    {.temperature = 750, .pwm = 25},
    {.temperature = 1000,.pwm = 50},
    {.temperature = 1250,.pwm = 200},
    {.temperature = 1500, .pwm = 250},
    {.temperature = 1750, .pwm = 300},
    {.temperature = 2000,.pwm = 320},
    {.temperature = 2500,.pwm = 340},
    {.temperature = 3000,.pwm = 380}
};

static const APP_FB_PID_PARAMETER_T heater_pid =
{
    /* Public/reference gains are defined at 20 ms. B4-T2 normalizes Ki/Kd
       internally when another fixed sample time is supplied at init. */
    .kp = 9000,
    .ki = 300,
    .kd = 5000,
    .integral_limit = 32767,
    .output_limit = 900,
    .kaw = APP_FB_PID_KAW_DEFAULT
};

APP_FB_TEMPERATURE_CONTROLLER_T heater_controller;

MY_API void Heater_Control_Init(void)
{
    (void)Heater_Control_InitExTimed(0, APP_FB_SAMPLE_TIME_DEFAULT_MS);
}

MY_API void Heater_Control_InitEx(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter)
{
    (void)Heater_Control_InitExTimed(
        adaptive_parameter,
        APP_FB_SAMPLE_TIME_DEFAULT_MS);
}

MY_API APP_FB_ERROR Heater_Control_InitTimed(uint32_t sample_time_ms)
{
    return Heater_Control_InitExTimed(0, sample_time_ms);
}

MY_API APP_FB_ERROR Heater_Control_InitExTimed(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter,
    uint32_t sample_time_ms)
{
    APP_FB_TIMING_PARAMETER_T timing;

    timing.sample_time_ms = sample_time_ms;

    return app_fb_temperature_controller_init_ex_timed(
        &heater_controller,
        heater_ff_table,
        (int32_t)(sizeof(heater_ff_table) / sizeof(APP_FB_FF_POINT_T)),
        &heater_pid,
        adaptive_parameter,
        &timing);
}

MY_API uint32_t Heater_GetSampleTimeMs(void)
{
    return app_fb_temperature_controller_get_sample_time_ms(
        &heater_controller);
}

MY_API void Heater_SetAdaptiveParameter(
    const APP_FB_ADAPTIVE_PARAMETER_T *adaptive_parameter)
{
    app_fb_temperature_controller_set_adaptive_parameter(
        &heater_controller,
        adaptive_parameter);
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

    if(output_pid_out == 0 ||
       output_ff_pwm == 0 ||
       output_ff_offset == 0 ||
       output_heater_pwm == 0)
    {
        return;
    }

    input.enable = APP_FB_TRUE;
    input.sv = input_sv;
    input.pv = input_pv;
    input.mode = APP_FB_MODE_AUTO;

    app_fb_temperature_controller_run(&heater_controller, &input, &output);

    *output_pid_out = output.pid_output;
    *output_ff_pwm = output.ff_pwm;
    *output_ff_offset = output.ff_offset;
    *output_heater_pwm = output.pwm;
}

#ifdef __cplusplus
}
#endif
