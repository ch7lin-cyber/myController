#include "Simulation_FF_Table.h"

/*
 * Inverse of the ControllPlant steady-state characteristic.
 * Values are rounded to the nearest controller PWM count (0.1%).
 */
const APP_FB_FF_POINT_T g_controllplant_sim_ff_table[] =
{
    { .temperature = 250,  .pwm =    0 },  /*  25.00 C ->   0.0% */
    { .temperature = 500,  .pwm =   73 },  /*  50.00 C ->   7.3% */
    { .temperature = 750,  .pwm =  146 },  /*  75.00 C ->  14.6% */
    { .temperature = 934,  .pwm =  200 },  /*  93.40 C ->  20.0% identified */
    { .temperature = 1000, .pwm =  236 },  /* 100.00 C ->  23.6% */
    { .temperature = 1250, .pwm =  370 },  /* 125.00 C ->  37.0% */
    { .temperature = 1300, .pwm =  397 },  /* 130.00 C ->  39.7% regression */
    { .temperature = 1492, .pwm =  500 },  /* 149.15 C ->  50.0% identified */
    { .temperature = 1500, .pwm =  522 },  /* 150.00 C ->  52.2% */
    { .temperature = 1550, .pwm =  650 },  /* 155.00 C ->  65.0% */
    { .temperature = 1600, .pwm =  778 },  /* 160.00 C ->  77.8% regression */
    { .temperature = 1608, .pwm =  800 },  /* 160.84 C ->  80.0% identified */
    { .temperature = 1650, .pwm =  907 },  /* 165.00 C ->  90.7% extrapolated */
    { .temperature = 1686, .pwm = 1000 }   /* 168.63 C -> 100.0% extrapolated */
};

const int32_t g_controllplant_sim_ff_table_size =
    (int32_t)(sizeof(g_controllplant_sim_ff_table) /
              sizeof(g_controllplant_sim_ff_table[0]));
