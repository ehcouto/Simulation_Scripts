/**
*
*
*/

#ifndef MATH_H
#define MATH_H

typedef struct{
	float rms; 
	float rms2; 
	float v2sum;

	unsigned cntr;
	unsigned prevTime;
} sRmsCalc_t;

typedef struct{
	float maxActual;
	float maxResult;
	unsigned prevTime;
} sMaxCalc_t;

typedef struct{
	float sum;
	float ave;
	unsigned cntr;
	unsigned prevTime;
} sAveCalc_t;

#define MC_PI           (3.14159265359f)
#define MC_SQRT_3       (1.732051f)
#define MC_TWOPI        (2.0f * MC_PI)
#define MC_RF2RPM       (60.0f/MC_TWOPI)
#define MC_RAD2DEG      (180.0f/MC_PI)
#define MC_PI2DEG       (180.0f)
#define MC_TWOPI2DEG    (360.0f)
#define MC_SQRT3D3      (MC_SQRT_3/3.0f)

#define RPM2RPS_DIVIDER (60.0f)

#define RPM2RADIAL(x,pp)   ((mcsp_t)MC_TWOPI * (mcsp_t)(pp) * (mcsp_t)(x) / (mcsp_t)MIN_TO_SEC_DIVIDER)

#define MC_FLOAT32_MIN                  (1.175494f * (float)(1.0e-38))
#define MC_FLOAT32_MAX                  (3.4028237f * (float)(1.0e+38))
#define MC_FLOAT32_VALUE_ONE_LESS       (1.0f - MC_FLOAT32_MIN)
#define MC_FLOAT32_VALUE_ONE_MORE       (1.0f + MC_FLOAT32_MIN)

float mathAbsoluteFlt(const float input);
float mathMergeFlt(float desired, float prev, float step, float *coeff);
float mathSinFlt(const float angle);
float mathCosFlt(const float angle);
void mathCalcSinCosFlt(const float angle, float *psin, float *pcos);
float mathTanFlt(const float angle);
float mathArcTanFlt(float ampsin, float ampcos);
float mathAngleSumDeg(const float l_angle_deg_1, const float l_angle_deg_2);
float mathCalculateRms(float value, sRmsCalc_t *rmsCalcObj, unsigned timeCnt);
float mathCalculateMax(float value, sMaxCalc_t *maxCalcObj, unsigned timeCnt);
float mathCalculateAve(float value, sAveCalc_t *aveCalcObj, unsigned timeCnt);

extern const mc_float_t sin_array[];
extern const mc_float_t cos_array[];
#endif
