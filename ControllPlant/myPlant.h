/******************************************************************************
 * File    : myPlant.h
 * Brief   : Thermal Plant Model identified from measured MV/PV data.
 *
 * Model:
 *   dT/dt = (T_eq(MV) - T) / tau(MV)
 *
 * Identified operating points: MV = 20, 50 and 80 %.
 * 0..80 % is the validated/identified model range.
 * 80..100 % is an extrapolated model range and must not be treated as
 * measured plant behavior until additional identification data is available.
 *
 * T_eq and tau were obtained by fitting each measured heating response to
 * a first-order thermal response:
 *
 *   PV(t) = PV0 + A * (1 - exp(-t/tau))
 *
 * This is a simulation model for controller development, not a replacement
 * for the physical thermal system.
 ******************************************************************************/

#ifndef MY_PLANT_H
#define MY_PLANT_H

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

#define THERMAL_PLANT_IDENTIFIED_MV_MAX_PERCENT   (80.0f)
#define THERMAL_PLANT_SIMULATION_MV_MAX_PERCENT   (100.0f)

typedef struct
{
    float temperature_c;       /* Current plant temperature [degC] */
    float ambient_c;           /* Ambient / zero-MV equilibrium [degC] */
    float sample_time_s;       /* Simulation step [s] */
    float tau_s;               /* Current thermal time constant [s] */
    float mv_percent;          /* Last applied MV [0..100 %] */
    bool  initialized;
} ThermalPlant_t;

/* Initialize the plant state. */
void ThermalPlant_Init(ThermalPlant_t *plant,
                       float initial_temperature_c,
                       float ambient_c,
                       float sample_time_s);

/* Reset plant temperature while keeping configuration. */
void ThermalPlant_Reset(ThermalPlant_t *plant,
                        float initial_temperature_c);

/* Set simulation/controller sample time. */
void ThermalPlant_SetSampleTime(ThermalPlant_t *plant,
                                float sample_time_s);

/*
 * Apply MV for one simulation step and return the new temperature [degC].
 * MV is expressed as percent: 0.0 .. 100.0.
 * Values above THERMAL_PLANT_IDENTIFIED_MV_MAX_PERCENT use extrapolated
 * equilibrium behavior.
 */
float ThermalPlant_Step(ThermalPlant_t *plant,
                        float mv_percent);

/* Return heating equilibrium characteristic for a given MV. */
float ThermalPlant_GetEquilibrium(float mv_percent);

/* Return identified/interpolated thermal time constant for a given MV. */
float ThermalPlant_GetTimeConstant(float mv_percent);

/* Convenience function for 0.1 degC integer representation. */
int16_t ThermalPlant_GetTemperature_x10(const ThermalPlant_t *plant);

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* MY_PLANT_H */
