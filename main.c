/******************************************************************************
 * File        : main.c
 * Description : Minimal application example for the adaptive temperature
 *               controller.
 *
 * Important:
 *   1. The application owns the scheduler period.
 *   2. sample_time_ms is passed once during initialization.
 *   3. The same period must be used when calling Heater_myAdptiveControl().
 *   4. Replace App_ReadTemperature(), App_GetSetpoint() and App_SetHeaterPwm()
 *      with the target platform's ADC/sensor, command and PWM interfaces.
 ******************************************************************************/

#include <stdint.h>
#include "PID_ControllerSrc/entry_C_adptiveTempController.h"

#define APP_CONTROL_SAMPLE_TIME_MS    (20U)
#define APP_PWM_MIN                   (0)
#define APP_PWM_MAX                   (1000)

/* --------------------------------------------------------------------------
 * Platform interface examples.
 * Replace these functions with the actual MCU/application implementation.
 * Temperature unit: 0.1 degree C
 * PWM unit        : 0.1 % (0..1000 = 0.0..100.0 %)
 * -------------------------------------------------------------------------- */
static int16_t App_ReadTemperature(void)
{
    /* Example: 25.0 C. Replace with actual sensor value. */
    return 250;
}

static int16_t App_GetSetpoint(void)
{
    /* Example: 130.0 C. Replace with HMI/communication/application SV. */
    return 1300;
}

static void App_SetHeaterPwm(int32_t pwm)
{
    /*
     * Replace with the target MCU PWM output function.
     * Example:
     *     BSP_PWM_SetDuty((uint16_t)pwm);
     */
    (void)pwm;
}

static int32_t App_ClampPwm(int32_t pwm)
{
    if (pwm < APP_PWM_MIN)
    {
        return APP_PWM_MIN;
    }

    if (pwm > APP_PWM_MAX)
    {
        return APP_PWM_MAX;
    }

    return pwm;
}

/* --------------------------------------------------------------------------
 * Controller task.
 * Call exactly once every APP_CONTROL_SAMPLE_TIME_MS.
 * -------------------------------------------------------------------------- */
static void App_ControllerTask(void)
{
    int16_t pv;
    int16_t sv;
    int32_t pid_out = 0;
    int32_t ff_pwm = 0;
    int32_t ff_offset = 0;
    int32_t heater_pwm;

    pv = App_ReadTemperature();
    sv = App_GetSetpoint();

    Heater_myAdptiveControl(
        pv,
        sv,
        &pid_out,
        &ff_pwm,
        &ff_offset);

    /*
     * Current wrapper exposes the three controller components separately.
     * Combine them here to obtain the final heater command.
     */
    heater_pwm = pid_out + ff_pwm + ff_offset;
    heater_pwm = App_ClampPwm(heater_pwm);

    App_SetHeaterPwm(heater_pwm);
}

int main(void)
{
    APP_FB_ERROR status;

    /*
     * MCU/platform initialization examples:
     *   SystemClock_Init();
     *   Sensor_Init();
     *   PWM_Init();
     *   Communication_Init();
     */

    /* Pass the actual fixed scheduler period once during initialization. */
    status = Heater_Control_InitTimed(APP_CONTROL_SAMPLE_TIME_MS);
    if (status != APP_FB_OK)
    {
        /* Invalid timing/configuration: keep heater output OFF. */
        App_SetHeaterPwm(0);

        for (;;)
        {
            /* Application fault handling. */
        }
    }

    for (;;)
    {
        /*
         * This example assumes this block executes every 20 ms.
         * In a real target, normally use an RTOS task, hardware timer,
         * scheduler tick, or application task dispatcher instead of a
         * blocking delay.
         */
        App_ControllerTask();

        /*
         * Example only:
         *     OS_DelayMs(APP_CONTROL_SAMPLE_TIME_MS);
         * or wait for the next periodic scheduler event.
         */
    }
}
