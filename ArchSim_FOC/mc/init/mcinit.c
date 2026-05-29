#include "mcinclude.h"


void mcParametersInit(mcCpv_t *p_mpv)
{
    p_mpv->p = *p_mpv->p_flash;
    p_mpv->parameterCheckFlag = MC_FALSE;

    if(p_mpv->p.modulation.speedR1 < 0.0f)
    {
        p_mpv->p.modulation.speedR1 = (-1.0f) * p_mpv->p.modulation.speedR1;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }

    if(p_mpv->p.sprefmax < 0)
    {
        p_mpv->p.sprefmax = (mcsp_t)(-1.0f) * p_mpv->p.sprefmax;
        p_mpv->parameterCheckFlag = MC_TRUE;
}

    if(p_mpv->p.minsp_c2o < 0)
    {
        p_mpv->p.minsp_c2o = (mcsp_t)(-1.0f) * p_mpv->p.minsp_c2o;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }

    if(p_mpv->p.hwHardBrake.maxsp < 0)
    {
        p_mpv->p.hwHardBrake.maxsp = (mcsp_t)(-1.0f) * p_mpv->p.hwHardBrake.maxsp;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }
    if(p_mpv->p.hwHardBrake.minsp < 0)
    {
        p_mpv->p.hwHardBrake.minsp = (mcsp_t)(-1.0f) * p_mpv->p.hwHardBrake.minsp;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }
    if(p_mpv->p.hwHardBrake.maxsp > p_mpv->p.sprefmax)
    {
        p_mpv->p.hwHardBrake.maxsp = p_mpv->p.sprefmax;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }
    if(p_mpv->p.hwHardBrake.minsp > p_mpv->p.hwHardBrake.maxsp)
    {
        p_mpv->p.hwHardBrake.minsp = p_mpv->p.hwHardBrake.maxsp;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }
    if (p_mpv->p.hwHardBrake.minsp <= p_mpv->p.minsp_c2o)
    {
        p_mpv->p.hwHardBrake.enabled = MC_FALSE;
        p_mpv->parameterCheckFlag = MC_TRUE;
    }
}

void mcVariablesInit(mcCpv_t *p_mpv)
{
    mc_uint32_t i;
    
    for(i=0u; i < sizeof(mcCv_t); i++)
    {
      *((mc_uint8_t*)&p_mpv->v + i) = 0u;
    }

    p_mpv->v.diffLdLq = p_mpv->p.phys.Ld - p_mpv->p.phys.Lq;
    
    p_mpv->v.piDCurrent.kp = p_mpv->p.piDCurrent.kp;
    p_mpv->v.piDCurrent.ki = p_mpv->p.piDCurrent.ki / (mcv_t)p_mpv->p.freqFL;
    p_mpv->v.piDCurrent.antiWindupCoef = p_mpv->p.piDCurrent.antiWindupCoef;
    p_mpv->v.piDCurrent.antiWindupLimit = p_mpv->p.piDCurrent.antiWindupLimit;
    p_mpv->v.piDCurrent.prop = 0.0f;
    p_mpv->v.piDCurrent.integ = 0.0f;
    p_mpv->v.vdq_decoupler_dq.d = 0.0f;

    p_mpv->v.piQCurrent.kp = p_mpv->p.piQCurrent.kp;
    p_mpv->v.piQCurrent.ki = p_mpv->p.piQCurrent.ki / (mcv_t)p_mpv->p.freqFL;
    p_mpv->v.piQCurrent.antiWindupCoef = p_mpv->p.piQCurrent.antiWindupCoef;
    p_mpv->v.piQCurrent.antiWindupLimit = p_mpv->p.piQCurrent.antiWindupLimit;
    p_mpv->v.piQCurrent.prop = 0.0f;
    p_mpv->v.piQCurrent.integ = 0.0f;
    p_mpv->v.vdq_decoupler_dq.q = 0.0f;
    
    p_mpv->v.decoupling_lpfcoeff.d = p_mpv->p.decoupling_lpfcoeff.d / (mcv_t)p_mpv->p.freqFL;
    if(p_mpv->v.decoupling_lpfcoeff.d > 1.0f)
        p_mpv->v.decoupling_lpfcoeff.d = 1.0f;
    p_mpv->v.decoupling_lpfcoeff.q = p_mpv->p.decoupling_lpfcoeff.q / (mcv_t)p_mpv->p.freqFL;
    if(p_mpv->v.decoupling_lpfcoeff.q > 1.0f)
        p_mpv->v.decoupling_lpfcoeff.q = 1.0f;
    p_mpv->v.inductanceCorrectionCoeff.d = p_mpv->p.inductanceCorrectionCoeff.d / (mcv_t)p_mpv->p.freqFL;
    if(p_mpv->v.inductanceCorrectionCoeff.d > 1.0f)
        p_mpv->v.inductanceCorrectionCoeff.d = 1.0f;
    p_mpv->v.inductanceCorrectionCoeff.q = p_mpv->p.inductanceCorrectionCoeff.q / (mcv_t)p_mpv->p.freqFL;
    if(p_mpv->v.inductanceCorrectionCoeff.q > 1.0f)
        p_mpv->v.inductanceCorrectionCoeff.q = 1.0f;

    // need to discuss
    p_mpv->v.periodFL = 1.0f / (period_t)p_mpv->p.freqFL;
    p_mpv->v.periodSL = 1.0f / (period_t)p_mpv->p.freqSL;

    p_mpv->v.spControllerRegion = MC_SP_CONTROLLER_REGION_1;
    p_mpv->v.piSpeedOverwrite = 0;
    p_mpv->v.piSpeed.kp = p_mpv->p.piSpeedR1.kp;
    p_mpv->v.piSpeed.ki = p_mpv->p.piSpeedR1.ki / (mcc_t)p_mpv->p.freqSL;
    p_mpv->v.piSpeed.antiWindupCoef = p_mpv->p.piSpeedR1.antiWindupCoef;
    p_mpv->v.piSpeed.antiWindupLimit = p_mpv->p.piSpeedR1.antiWindupLimit;
    p_mpv->v.piSpeed.prop = (mcc_t)0;
    p_mpv->v.piSpeed.integ = (mcc_t)0;

    p_mpv->v.fwv.pi.kp = p_mpv->p.fwp.pi.kp;
    p_mpv->v.fwv.pi.ki = p_mpv->p.fwp.pi.ki / (mcc_t)p_mpv->p.freqFL;
    p_mpv->v.fwv.pi.antiWindupCoef = p_mpv->p.fwp.pi.antiWindupCoef;
    p_mpv->v.fwv.pi.antiWindupLimit = p_mpv->p.fwp.pi.antiWindupLimit;
    p_mpv->v.fwv.pi.prop = 0.0f;
    p_mpv->v.fwv.pi.integ = 0.0f;
    p_mpv->v.fwv.vmarg_filtered = 0.0f;
    p_mpv->v.fwv.vamp_lpfcoeff = p_mpv->p.fwp.vamp_lpfcoeff / (mcv_t)p_mpv->p.freqFL;
    if(p_mpv->v.fwv.vamp_lpfcoeff > 1.0f)
        p_mpv->v.fwv.vamp_lpfcoeff = 1.0f;
    p_mpv->v.fwv.vmarg_lpfcoeff = p_mpv->p.fwp.vmarg_lpfcoeff / (mcv_t)p_mpv->p.freqFL;
    if(p_mpv->v.fwv.vmarg_lpfcoeff > 1.0f)
        p_mpv->v.fwv.vmarg_lpfcoeff = 1.0f;

    p_mpv->v.mtpa.idref = (mcc_t)0.0f;
    
    if(p_mpv->p.rampParams.rampmaxDec > p_mpv->p.rampParams.rampmaxAcc)
        p_mpv->v.rampin = p_mpv->p.rampParams.rampmaxAcc;
    else
        p_mpv->v.rampin = p_mpv->p.rampParams.rampmaxDec;
    
    p_mpv->v.transition.ramp = 0.0f;
    
    p_mpv->v.vab.a = p_mpv->v.vab.b = 0.0f;
    p_mpv->v.idq.d = p_mpv->v.idq.q = 0.0f;
    p_mpv->v.idqref.d = p_mpv->v.idqref.q = 0.0f;
    p_mpv->v.duvw.u = p_mpv->v.duvw.v = p_mpv->v.duvw.w = 0.0f;
    p_mpv->v.duvw_comp.u = p_mpv->v.duvw_comp.v = p_mpv->v.duvw_comp.w = 0.0f;
    
    p_mpv->v.just_powered_delay = 0u;
    p_mpv->v.calready = MC_FALSE;
    
    p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_PASSIVE;
    
    p_mpv->v.rot = ROT_NO;
    p_mpv->v.acc = ACC_NO;
    
    p_mpv->v.hwHardBrake.state = MC_HW_HB_STATE_PASSIVE;
    
    p_mpv->p.hwHardBrake.minsp = mathAbsoluteFlt(p_mpv->p.hwHardBrake.minsp);
    p_mpv->p.hwHardBrake.maxsp = mathAbsoluteFlt(p_mpv->p.hwHardBrake.maxsp);
    
    p_mpv->v.bypass.spController = MC_FALSE;
}

void mcBootstrapStart(mcCpv_t *p_mpv)
{
    //p_mpv->p.btstrp.duration = 200u;//BOOTSTRAP_STEP_FREQ_FL;
    p_mpv->v.btstrp.step = BOOTSTRAP_INITIATED;
}
