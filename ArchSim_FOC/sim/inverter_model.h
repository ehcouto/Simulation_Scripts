#ifndef INVERTER_MODEL_H
#define INVERTER_MODEL_H

#include "C_Extensions.h"

typedef struct {
    float32 Va;
    float32 Vb;
    float32 Vc;
} PhaseVoltages;

typedef struct {
    float32 duty_u;  //Duty Cycle (U) - adm
    float32 duty_v;  //Duty Cycle (V) - adm
    float32 duty_w;  //Duty Cycle (W) - adm
    float32 i_u;     //Phase Current (U) - Amps
    float32 i_v;     //Phase Current (V) - Amps
    float32 i_w;     //Phase Current (W) - Amps
    float32 vdc;     //DC Bus Voltage - Volts
} InverterInput;


void inverter_model_init(float32 Tpwm);
PhaseVoltages inverter_model_ideal(InverterInput inv_in);
PhaseVoltages inverter_model_dt_losses(InverterInput inv_in);
PhaseVoltages inverter_model_delay(InverterInput inv_in);

#endif
