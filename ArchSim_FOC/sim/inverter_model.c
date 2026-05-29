#include "inverter_model.h"

#define SQRT3   1.73205080757f
#define MATHCALC__SATURATE_DIRECT(lim_inf, value, lim_sup)                                          \
 (((value) >= (lim_sup)) ? (lim_sup) : ( ((value) <= (lim_inf)) ? (lim_inf) : (value) ))            \

float32 VMax;

 void inverter_init(float32 vmax)
 {
     VMax = vmax;
 }

void inverter_step(float32 valpha, float32 vbeta, float32 *a_out, float32 *b_out, float32 *c_out)
{
    float32 temp;

	temp =  valpha;
    *a_out = MATHCALC__SATURATE_DIRECT(-VMax,temp,VMax);

    temp = (-valpha + SQRT3 * vbeta) / 2.0;
    *b_out = MATHCALC__SATURATE_DIRECT(-VMax,temp,VMax);

    temp = (-valpha - SQRT3 * vbeta) / 2.0;
    *c_out = MATHCALC__SATURATE_DIRECT(-VMax,temp,VMax);
}