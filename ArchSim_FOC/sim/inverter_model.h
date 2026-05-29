#ifndef INVERTER_MODEL_H
#define INVERTER_MODEL_H

#include "C_Extensions.h"

void inverter_init(float32 vmax);
void inverter_step(float32 valpha, float32 vbeta, float32 *a_out, float32 *b_out, float32 *c_out);

#endif
