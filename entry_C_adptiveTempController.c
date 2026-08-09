
/***************************************************************
Description : 
	This is a user C test Main program application.
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
    .kp = 9000,
    .ki = 300,
    .kd = 5000,
    .integral_limit = 5000,
    .output_limit = 450,
    .kaw = APP_FB_PID_KAW_DEFAULT
};

APP_FB_TEMPERATURE_CONTROLLER_T heater_controller;

MY_API void Heater_Control_Init(void)
{
    app_fb_temperature_controller_init(&heater_controller, heater_ff_table, sizeof(heater_ff_table) / sizeof(APP_FB_FF_POINT_T), &heater_pid);
}

MY_API void Heater_myAdptiveControl(int16_t input_pv, int16_t input_sv, int32_t *output_pid_out, int32_t *output_ff_pwm, int32_t *output_ff_offset)
{
    static APP_FB_TEMP_CONTROLLER_INPUT_T input;
    static APP_FB_TEMP_CONTROLLER_OUTPUT_T output;
    input.enable = APP_FB_TRUE;
    input.sv = input_sv;
    input.pv = input_pv;
    input.mode = APP_FB_MODE_AUTO;
    app_fb_temperature_controller_run(&heater_controller, &input, &output);
    *output_pid_out = output.pid_output;
    *output_ff_pwm = output.ff_pwm;
    *output_ff_offset = output.ff_offset;
}

#define PC_SIMULATION

#ifdef PC_SIMULATION
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif
#define SAMPLE_TIME_MS (20)
#define TEST_TIME_SEC (20)
#define SAMPLE_COUNT ((TEST_TIME_SEC * 1000) / SAMPLE_TIME_MS)
#define SINE_PERIOD_SEC (10.0)
#define PV_MIN (500)
#define PV_MAX (1750)
#define SV_VALUE (1300)

int main(void)
{
    int i;
    int16_t pv;
    int16_t sv = SV_VALUE;
    int32_t pid_out;
    int32_t ff_pwm;
    int32_t ff_offset;
    double angle;
    double pv_center;
    double pv_amp;
    double sample_per_cycle;
    FILE *fp;

    fp = fopen("simulation.csv", "w");
    if(fp == NULL)
    {
        printf("Cannot create simulation.csv\n");
        return -1;
    }
    fprintf(fp, "time_ms,pv,sv,error,pid_out,ff_pwm,ff_offset,total_output\n");
    Heater_Control_Init();
    pv_center = (PV_MAX + PV_MIN) / 2.0;
    pv_amp = (PV_MAX - PV_MIN) / 2.0;
    sample_per_cycle = (SINE_PERIOD_SEC * 1000.0) / SAMPLE_TIME_MS;
    for(i = 0; i < SAMPLE_COUNT; i++)
    {
        angle = 2.0 * PI * (double)i / sample_per_cycle;
        pv = (int16_t)(pv_center + pv_amp * sin(angle));
        Heater_myAdptiveControl(pv, sv, &pid_out, &ff_pwm, &ff_offset);
        fprintf(fp, "%d,%d,%d,%d,%ld,%ld,%ld,%ld\n", i * SAMPLE_TIME_MS, pv, sv, sv - pv, (long)pid_out, (long)ff_pwm, (long)ff_offset, (long)(pid_out + ff_pwm + ff_offset));
    }
    fclose(fp);
    printf("Simulation completed.\n");
    printf("Output file : simulation.csv\n");
    printf("Samples     : %d\n", SAMPLE_COUNT);
    return 0;
}
#endif

#ifdef __cplusplus
}
#endif
