/**
*
*
*/

#ifndef MC_H
#define MC_H

/** \defgroup mc_def_pre_stat Motor control; defined preprocessor statements
* @{
*/

/** @}*/

/** \defgroup mc_def_pre_const Motor control; defined preprocessor constants
* @{
*/
#define REQUESTED_TOTAL_MOTOR_SUPPORT           (2u)       /* read only */

#define MOTOR1_FAST_LOOP_FREQUENCY      8000u
#define MOTOR1_SLOW_LOOP_FREQUENCY      1000u

#define MOTOR2_FAST_LOOP_FREQUENCY      7550u
#define MOTOR2_SLOW_LOOP_FREQUENCY      1000u
/** @}*/

/******************************************************************************/
/* included files */

#include <stdint.h>
#include <stdbool.h>

/* included files */
/******************************************************************************/

/******************************************************************************/
/** \defgroup mc_def_types defined types for other modules to use motor control module
* @{
*/
typedef struct{
    uint8_t version[4];
    uint8_t total_motor_supported;

    void (*mcDrvTurnOnInrushRelay)(void);
} mcInit_t;

typedef struct{
    uint32_t freqFL;
    uint32_t freqSL;
    
    void *p_flash;

    void (*mcBrdPwmEnable)(void);
    void (*mcBrdPwmDisable)(void);
    
    void (*mcBrdPwmEnablePhaseU)(void);
    void (*mcBrdPwmEnablePhaseV)(void);
    void (*mcBrdPwmEnablePhaseW)(void);
    void (*mcBrdPwmEnablePhaseUVW)(void);
    
    void (*mcBrdMapAdcChannels)(uint8_t sec);

    void (*mcBrdSampleCurrentsUVW)(uint8_t sec);
    float (*mcBrdGetPhaseCurrentU)(void);
    float (*mcBrdGetPhaseCurrentV)(void);
    float (*mcBrdGetPhaseCurrentW)(void);
    bool (*mcBrdCurrentCalibrationInit)(void);
    bool (*mcBrdCurrentCalibrationHandler)(void);

    float (*mcBrdGetDcBusVoltage)(void);
    bool (*mcBrdGetDCBusOpenStatus)(void);
    
    float (*mcBrdGetIpmTemperature)(void);
    bool (*mcBrdGetIpmFaultOutState)(void);
    void (*mcBrdResetIpmFaultOutState)(void);

    void (*mcBrdSetPwmDuties)(float dutyU, float dutyV, float dutyW);
    void (*mcBrdShortCircBotTransistors)(void);

    bool (*mcBrdGetEncoderData)(float *_pos, float *_sp);
} mcMpvInit_t;

typedef struct{
    int8_t mc_motorIndex;
    float mc_sprefmec;
    float mc_ramp_duration;    
    float mc_rampin;
} McSpeedReq_t;
/** @}*/
/******************************************************************************/

/******************************************************************************/
/* function declerations */

void mcInitMotorControl(mcInit_t *l_mcInit);
void mcAddMotor(const mcMpvInit_t *l_mcMpvInit);

bool mcRequestSpeed(const McSpeedReq_t *l_spreq);
bool mcRequestStop(const int32_t l_MxIndex);
float mcGetPosition(const int32_t l_MxIndex, float *l_position);
bool mcGetIdq(const int32_t l_MxIndex, float *id, float *iq);
float mcGetId(const int32_t l_MxIndex);
float mcGetIq(const int32_t l_MxIndex);
float mcGetVd(const int32_t l_MxIndex);
float mcGetVq(const int32_t l_MxIndex);
float mcGetSprefAbs(const int32_t l_MxIndex);
float mcGetSprefInAbs(const int32_t l_MxIndex);
void mcSetSpeedRamp(const int32_t l_MxIndex, float l_ramp);
float mcGetSpeedRamp(const int32_t l_MxIndex);
bool mcGetIdqref(const int32_t l_MxIndex, float *idref, float *iqref);
float mcGetIdref(const int32_t l_MxIndex);
float mcGetIqref(const int32_t l_MxIndex);
float mcGetIabVector(const int32_t l_MxIndex);
float mcGetActMecSpeed(const int32_t l_MxIndex);
float mcGetReqMecSpeed(const int32_t l_MxIndex);
float mcGetReqSpeed(const int32_t l_MxIndex);
float mcGetEstSpeed(const int32_t l_MxIndex);
float mcGetActSpeed(const int32_t l_MxIndex);
float mcGetRefSpeed(const int32_t l_MxIndex);
bool mcGetEstPosition(const int32_t l_MxIndex, float *l_thest);
signed char mcGetMotorDirection(const int32_t l_MxIndex);
float mcGetBeltRatio(const int32_t l_MxIndex);
float mcGetSprefMax(const int32_t l_MxIndex, float *l_spref);
float mcGetBusVoltage(const int32_t l_MxIndex);
float mcGetLd(const int32_t l_MxIndex);
float mcGetLq(const int32_t l_MxIndex);
int8_t mcIsStateFault(const int32_t l_MxIndex);
int8_t mcIsStateRamp(const int32_t l_MxIndex);
float mcGetfsFatalFaultFlag(const int32_t l_MxIndex);
float mcGetStopRPMValue(const int32_t l_MxIndex);
void mcSetSpeedControllerParameters(int32_t l_MxIndex, bool l_overwrite, float l_kp, float l_ki);
void mcSetSpeedControllerInteg(int32_t l_MxIndex, float l_sp_integ);
void mcBypassSpController(int32_t l_MxIndex, bool l_bypass, float l_iqref, float l_spref);
void mcGetDCurrentControlParameters(int32_t l_MxIndex, float *l_kp, float *l_ki);
void mcGetQCurrentControlParameters(int32_t l_MxIndex, float *l_kp, float *l_ki);
void mcSetDCurrentControlParameters(int32_t l_MxIndex, float l_kp, float l_ki);
void mcSetQCurrentControlParameters(int32_t l_MxIndex, float l_kp, float l_ki);
void mcEnableMotorControl(int32_t l_MxIndex);
void mcDisableMotorControl(int32_t l_MxIndex);
bool mcGetMotorControlState(int32_t l_MxIndex);
void mcActivateFunctionalsafety(int32_t l_MxIndex);
uint32_t mcGetGeneratedFaultsBinary(int32_t l_MxIndex);

void mcMxHandlerFL(int32_t l_MxIndex);
void mcMxHandlerSL(int32_t l_MxIndex);
void mcHandlerML(void);

/* function declerations */
/******************************************************************************/

#endif
