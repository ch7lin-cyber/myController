#ifndef CONTROLL_PLANT_SIMULATION_FF_TABLE_H_
#define CONTROLL_PLANT_SIMULATION_FF_TABLE_H_

#include <stdint.h>
#include "../PID_ControllerSrc/FB_C_feedforward_table.h"

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
extern "C" {
#endif
//------------------------------------------------------------------------------------//

/*
 * Simulation feedforward table derived from the identified ControllPlant
 * steady-state characteristic T_eq(MV).
 *
 * Controller temperature unit : 0.1 degC
 * Controller PWM unit         : 0.1 % (0..1000 == 0..100%)
 *
 * 0..80% MV points are based on identified data.
 * 80..100% is extrapolated from the identified 50..80% segment, matching
 * ThermalPlant_GetEquilibrium().
 */

extern const APP_FB_FF_POINT_T g_controllplant_sim_ff_table[];
extern const int32_t g_controllplant_sim_ff_table_size;

//------------------------------------------------------------------------------------//
// C++ compatibility - DO NOT DELETE
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------------//

#endif /* CONTROLL_PLANT_SIMULATION_FF_TABLE_H_ */
