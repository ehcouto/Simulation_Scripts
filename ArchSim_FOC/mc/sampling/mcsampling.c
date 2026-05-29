#include "mcinclude.h"


static void mcSamplePhaseCurrents(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func);
static void mcSampleDcBusVoltage(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func);
static void mcGetIpmTemperature(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func);
static void mcGetIpmFaultOutPin(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func);


void mcSamplingHandler(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func)
{
    mcSamplePhaseCurrents(p_mpv, p_spv, p_func);
    mcSampleDcBusVoltage(p_mpv, p_spv, p_func);
    mcGetIpmTemperature(p_mpv, p_spv, p_func);
    mcGetIpmFaultOutPin(p_mpv, p_spv, p_func);
}

static void mcSamplePhaseCurrents(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func)
{
    mcuvw_t _iuvw;
    mcab_t _iab;
    mcc_t _iamp;
    mcc_t _iamp2;

    p_func->mcBrdSampleCurrentsUVW(p_mpv->v.sector);
    
    _iuvw.u = p_func->mcBrdGetPhaseCurrentU();
    _iuvw.v = p_func->mcBrdGetPhaseCurrentV();
    _iuvw.w = p_func->mcBrdGetPhaseCurrentW();
    ClarkTransform(&_iuvw, &_iab);
    _iamp2 = (_iab.a * _iab.a) + (_iab.b * _iab.b);
    _iamp = (mcc_t)sqrtf(_iamp2);

    p_mpv->v.iuvw.u = _iuvw.u;
    p_mpv->v.iuvw.v = _iuvw.v;
    p_mpv->v.iuvw.w = _iuvw.w;
    p_mpv->v.iab = _iab;
    p_mpv->v.iamp2 = _iamp2;
    p_mpv->v.iamp = _iamp;

    p_spv->v.iuvw.u = _iuvw.u;
    p_spv->v.iuvw.v = _iuvw.v;
    p_spv->v.iuvw.w = _iuvw.w;
    p_spv->v.iuvw_abs.u = mathAbsoluteFlt(_iuvw.u);
    p_spv->v.iuvw_abs.v = mathAbsoluteFlt(_iuvw.v);
    p_spv->v.iuvw_abs.w = mathAbsoluteFlt(_iuvw.w);
    p_spv->v.iab = _iab;
    p_spv->v.iamp2 = _iamp2;
    p_spv->v.iamp = _iamp;
}

static void mcSampleDcBusVoltage(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func)
{
    mcv_t _vbus;

    _vbus = p_func->mcBrdGetDcBusVoltage();

    p_mpv->v.vbus = _vbus;
    p_mpv->v.vbus_filtered += 0.05f * (p_mpv->v.vbus - p_mpv->v.vbus_filtered);

    p_spv->v.vbus_pin_circuit_state = p_func->mcBrdGetDCBusOpenStatus();
    p_spv->v.vbus = _vbus;
}

static void mcGetIpmFaultOutPin(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func)
{
    p_spv->v.ipm_faultout = p_func->mcBrdGetIpmFaultOutState();
    p_func->mcBrdResetIpmFaultOutState();
}

static void mcGetIpmTemperature(mcCpv_t *p_mpv, fsSpv_t *p_spv, mcMotorFunc_t *p_func)
{
    p_spv->v.ipm_temp_sampled = p_func->mcBrdGetIpmTemperature();
}
