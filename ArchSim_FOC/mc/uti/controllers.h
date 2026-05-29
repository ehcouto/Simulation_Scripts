#ifndef CONTROLLERS_H
#define CONTROLLERS_H


typedef mc_float_t contPiKp_t;       ///< pi controller Kp parameter 
typedef mc_float_t contPiKi_t;       ///< pi controller Ki parameter
typedef mc_float_t contPiPro_t;      ///< pi controller proportional component 
typedef mc_float_t contPiInteg_t;    ///< pi controller integral component

typedef struct {
    mc_float_t error;
    contPiKp_t kp;
    contPiKi_t ki;
    contPiPro_t prop;
    contPiInteg_t integ;
    mc_float_t antiWindupCoef;
    mc_float_t antiWindupLimit;
} contPiController_t;    ///< variable type for pi controller variables


mc_float_t contDiscretePiController(mc_float_t _error, contPiController_t* _piv);
mc_float_t contSaturate(mc_float_t value, mc_float_t vmin, mc_float_t vmax);
mc_float_t contCheckAngle0to360deg(mc_float_t value);

#endif