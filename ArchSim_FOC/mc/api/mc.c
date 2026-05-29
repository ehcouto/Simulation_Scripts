/**
*
*
*/


#include <stdint.h>
#include <stdbool.h>
#include "mcinclude.h"


char *MCP_VERSION = "AR_MCP_000_000_000_001";

mcMotorFunc_t func[SUPPORTED_TOTAL_MOTOR];         ///< functions used by a specific motor


/************************ Function definitions - start ************************/
void mcInitMotorControl(mcInit_t *l_mcInit)
{
    if(l_mcInit == MC_NULL)
    {
        ;       /* do nothing */
    }
    else
    {
        l_mcInit->version[0] = (100 * (MCP_VERSION[19] - '0')) + (10 * (MCP_VERSION[20] - '0')) + (MCP_VERSION[21] - '0');
        l_mcInit->version[1] = (100 * (MCP_VERSION[15] - '0')) + (10 * (MCP_VERSION[16] - '0')) + (MCP_VERSION[17] - '0');
        l_mcInit->version[2] = (100 * (MCP_VERSION[11] - '0')) + (10 * (MCP_VERSION[12] - '0')) + (MCP_VERSION[13] - '0');
        l_mcInit->version[3] = (100 * (MCP_VERSION[7] - '0')) + (10 * (MCP_VERSION[8] - '0')) + (MCP_VERSION[9] - '0');
        
        l_mcInit->total_motor_supported = SUPPORTED_TOTAL_MOTOR;
        
        mc.func.mcDrvTurnOnInrushRelay = l_mcInit->mcDrvTurnOnInrushRelay;
        
        mc.motorIndex = 0;
        
        mc.Initialized = MC_TRUE;
    }
}

void mcAddMotor(const mcMpvInit_t *l_mcMpvInit)
{
    mc_int32_t **l_pCurrentFuncMax;
    mc_int32_t **l_pCurrentFunc;
    
    /* cases not to add motor */ 
    if((mc.Initialized != MC_TRUE) || (l_mcMpvInit == MC_NULL) || 
       (mc.motorIndex >= (mindx_t)SUPPORTED_TOTAL_MOTOR) || 
           (l_mcMpvInit->freqFL <= l_mcMpvInit->freqSL))
    {
        ;       /* do nothing */
    }
    else
    {
        mpv[mc.motorIndex].mindx = mc.motorIndex;
        
        if(mcp_px[mpv[mc.motorIndex].mindx] != MC_NULL)
        {
            mpv[mc.motorIndex].p_flash = mcp_px[mpv[mc.motorIndex].mindx];
        }
        else
        {
            ;   /* do nothing */
        }

        func[mc.motorIndex].mcBrdPwmEnable = l_mcMpvInit->mcBrdPwmEnable;
        func[mc.motorIndex].mcBrdPwmDisable = l_mcMpvInit->mcBrdPwmDisable;
        func[mc.motorIndex].mcBrdPwmEnablePhaseU = l_mcMpvInit->mcBrdPwmEnablePhaseU;
        func[mc.motorIndex].mcBrdPwmEnablePhaseV = l_mcMpvInit->mcBrdPwmEnablePhaseV;
        func[mc.motorIndex].mcBrdPwmEnablePhaseW = l_mcMpvInit->mcBrdPwmEnablePhaseW;
        func[mc.motorIndex].mcBrdPwmEnablePhaseUVW = l_mcMpvInit->mcBrdPwmEnablePhaseUVW;
        func[mc.motorIndex].mcBrdMapAdcChannels = l_mcMpvInit->mcBrdMapAdcChannels;
        func[mc.motorIndex].mcBrdSampleCurrentsUVW = l_mcMpvInit->mcBrdSampleCurrentsUVW;
        func[mc.motorIndex].mcBrdGetPhaseCurrentU = l_mcMpvInit->mcBrdGetPhaseCurrentU;
        func[mc.motorIndex].mcBrdGetPhaseCurrentV = l_mcMpvInit->mcBrdGetPhaseCurrentV;
        func[mc.motorIndex].mcBrdGetPhaseCurrentW = l_mcMpvInit->mcBrdGetPhaseCurrentW;
        func[mc.motorIndex].mcBrdCurrentCalibrationInit = (mc_bool_t(*)(void))l_mcMpvInit->mcBrdCurrentCalibrationInit;
        func[mc.motorIndex].mcBrdCurrentCalibrationHandler = (mc_bool_t(*)(void))l_mcMpvInit->mcBrdCurrentCalibrationHandler;
        func[mc.motorIndex].mcBrdGetDcBusVoltage = l_mcMpvInit->mcBrdGetDcBusVoltage;
        func[mc.motorIndex].mcBrdGetDCBusOpenStatus = (mc_bool_t(*)(void))l_mcMpvInit->mcBrdGetDCBusOpenStatus;
        func[mc.motorIndex].mcBrdGetIpmTemperature = l_mcMpvInit->mcBrdGetIpmTemperature;
        func[mc.motorIndex].mcBrdGetIpmFaultOutState = (mc_bool_t(*)(void))l_mcMpvInit->mcBrdGetIpmFaultOutState;
        func[mc.motorIndex].mcBrdResetIpmFaultOutState = l_mcMpvInit->mcBrdResetIpmFaultOutState;
        func[mc.motorIndex].mcBrdSetPwmDuties = l_mcMpvInit->mcBrdSetPwmDuties;
        func[mc.motorIndex].mcBrdGetEncoderData = (mc_bool_t(*)(mc_float_t*, mc_float_t*))l_mcMpvInit->mcBrdGetEncoderData;
        func[mc.motorIndex].mcBrdShortCircBotTransistors = l_mcMpvInit->mcBrdShortCircBotTransistors;
        
        mpv[mc.motorIndex].func = &func[mc.motorIndex];

        l_pCurrentFunc = (mc_int32_t **)&mpv[mc.motorIndex].func;
        l_pCurrentFuncMax = l_pCurrentFunc + (sizeof(mpv[mc.motorIndex].func) / sizeof(mc_int32_t*));
        while(l_pCurrentFunc < l_pCurrentFuncMax)
        {
            if(*l_pCurrentFunc == (mc_int32_t*)0)
            {
                return;
            }
            else
            {
                l_pCurrentFunc += 1;
            }
        }
        
        mcParametersInit(&mpv[mc.motorIndex]);
        fsAddMxFuncSafety(&mpv[mc.motorIndex], &spv[mc.motorIndex], fsp_px[mpv[mc.motorIndex].mindx], &func[mc.motorIndex]);
        
        mcActivateFunctionalsafety(mc.motorIndex);
        mcEnableMotorControl(mc.motorIndex);
        mpv[mc.motorIndex].v.just_powered_delay = 0;

        mc.motorIndex++;
    }
}

bool mcRequestSpeed(const McSpeedReq_t *l_spreq)
{
    if(mpv[l_spreq->mc_motorIndex].control_state != MC_CONTROL_STATE_ACTIVE)
    {
        return MC_FALSE;
    }
    if(l_spreq->mc_motorIndex < mc.motorIndex)
    {
        mpv[l_spreq->mc_motorIndex].v.sprefmec = l_spreq->mc_sprefmec;
        
        if(l_spreq->mc_rampin != 0.0f)
        {
            mpv[l_spreq->mc_motorIndex].v.rampin = (rmps_t)l_spreq->mc_rampin;
            mpv[l_spreq->mc_motorIndex].v.ramp_duration = 0.0f;
        }
        else if(l_spreq->mc_ramp_duration == 0.0f)
        {
            mpv[l_spreq->mc_motorIndex].v.rampin = mpv[l_spreq->mc_motorIndex].p.rampParams.rampmaxAcc;
            mpv[l_spreq->mc_motorIndex].v.ramp_duration = 0.0f;
        }
        else
        {
            mpv[l_spreq->mc_motorIndex].v.ramp_duration = l_spreq->mc_ramp_duration;
        }
    }
    else
    {
        /* do nothing */
    }
    
    return MC_TRUE;
}

bool mcRequestStop(const int32_t l_MxIndex)
{
    if(l_MxIndex < mc.motorIndex)
    {
        mpv[l_MxIndex].v.ramp_duration = 0.0f;
        mpv[l_MxIndex].v.sprefmec = 0.0f;
    }
    else
    {
        /* do nothing */
    }
    
    return MC_TRUE;
}

float mcGetPosition(const int32_t l_MxIndex, float *l_position)
{
    if((mindx_t)l_MxIndex < mc.motorIndex)
    {
        *l_position = (float)mpv[(mindx_t)l_MxIndex].v.thact.th;
    }
    else
    {
        *l_position = 0.0f;
    }
    
    return *l_position;
}

bool mcGetIdq(const int32_t l_MxIndex, float *id, float *iq)
{
    *id = mpv[(mindx_t)l_MxIndex].v.idq.d;
    *iq = mpv[(mindx_t)l_MxIndex].v.idq.q;

    return MC_TRUE;
}

float mcGetId(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.idq.d;
}

float mcGetIq(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.idq.q;    
}

float mcGetVd(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.vdq.d;
}

float mcGetVq(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.vdq.q;    
}
/* */

/*   TG */
float mcGetSprefInAbs(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.sprefin_abs;
}

float mcGetSprefAbs(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.spref_abs;
}

void mcSetSpeedRamp(const int32_t l_MxIndex, float l_ramp)
{
    mpv[l_MxIndex].v.ramp = l_ramp;
}

float mcGetSpeedRamp(const int32_t l_MxIndex)
{
  return mpv[(mindx_t)l_MxIndex].v.ramp;
}

bool mcGetIdqref(const int32_t l_MxIndex, float *idref, float *iqref)
{
    *idref = mpv[(mindx_t)l_MxIndex].v.idqref.d;
    *iqref = mpv[(mindx_t)l_MxIndex].v.idqref.q;

    return MC_TRUE;
}

float mcGetIdref(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.idqref.d;
}

float mcGetIqref(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.idqref.q;    
}

float mcGetIabVector(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.iamp;
}

float mcGetActMecSpeed(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.spactmec;
}

float mcGetReqMecSpeed(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.sprefmec;
}

float mcGetReqSpeed(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.sprefin;
}

float mcGetEstSpeed(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.spest;
}

float mcGetActSpeed(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.spact;
}

float mcGetRefSpeed(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.spref;
}

bool mcGetEstPosition(const int32_t l_MxIndex, float *l_thest)
{
    *l_thest = (float)mpv[(mindx_t)l_MxIndex].v.thest.th;

    return MC_TRUE;
}

signed char mcGetMotorDirection(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.acc;
}

float mcGetBeltRatio(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].p.bratio;
}

float mcGetSprefMax(const int32_t l_MxIndex, float *l_spref)
{
    if((mindx_t)l_MxIndex < mc.motorIndex)
    {
        *l_spref = mpv[(mindx_t)l_MxIndex].p.sprefmax;
    }
    else
    {
        *l_spref = (mc_float_t)0;
    }
    
    return *l_spref;
}

float mcGetBusVoltage(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].v.vbus_filtered;
}

int8_t mcIsStateFault(const int32_t l_MxIndex)
{
    if(mpv[(mindx_t)l_MxIndex].v.state == FAULT)
    {
        return MC_TRUE;
    }
    else
    {
        return MC_FALSE;
    }
}

int8_t mcIsStateRamp(const int32_t l_MxIndex)
{
    if(mpv[(mindx_t)l_MxIndex].v.state == RAMP)
    {
        return MC_TRUE;
    }
    else
    {
        return MC_FALSE;
    }
}

void mcGetDCurrentControlParameters(int32_t l_MxIndex, float *l_kp, float *l_ki)
{
    *l_kp = mpv[(mindx_t)l_MxIndex].v.piDCurrent.kp;
    *l_ki = mpv[(mindx_t)l_MxIndex].v.piDCurrent.ki;
}

void mcGetQCurrentControlParameters(int32_t l_MxIndex, float *l_kp, float *l_ki)
{
    *l_kp = mpv[(mindx_t)l_MxIndex].v.piQCurrent.kp;
    *l_ki = mpv[(mindx_t)l_MxIndex].v.piQCurrent.ki;
}

void mcSetDCurrentControlParameters(int32_t l_MxIndex, float l_kp, float l_ki)
{
    mpv[(mindx_t)l_MxIndex].v.piDCurrent.kp = l_kp;
    mpv[(mindx_t)l_MxIndex].v.piDCurrent.ki = l_ki;
}

void mcSetQCurrentControlParameters(int32_t l_MxIndex, float l_kp, float l_ki)
{
    mpv[(mindx_t)l_MxIndex].v.piQCurrent.kp = l_kp;
    mpv[(mindx_t)l_MxIndex].v.piQCurrent.ki = l_ki;
}

void mcMxHandlerFL(int32_t l_MxIndex)
{
    if((mindx_t)l_MxIndex < mc.motorIndex)
    {
        mcSamplingHandler(&mpv[(mindx_t)l_MxIndex], &spv[(mindx_t)l_MxIndex], &func[(mindx_t)l_MxIndex]);
        
        fsFastLoopHandlerMx(&spv[(mindx_t)l_MxIndex]);
        
        if(spv[(mindx_t)l_MxIndex].v.mc_fs_state == FS_STATE_FAULT)
        {
            func[(mindx_t)l_MxIndex].mcBrdPwmDisable();
            mpv[(mindx_t)l_MxIndex].v.pwm_enabled = MC_FALSE;
            mpv[(mindx_t)l_MxIndex].v.state = FAULT;
            mpv[(mindx_t)l_MxIndex].v.spact = 0;
    }
    else
    {
            mcFastLoopHandlerMx(&mpv[l_MxIndex]);
        }
    }
    else
    {
        /* do nothing */
    }
}

void mcMxHandlerSL(int32_t l_MxIndex)
{
    if((mindx_t)l_MxIndex < mc.motorIndex)
    {
        fsSlowLoopHandlerMx(&spv[(mindx_t)l_MxIndex]);
        
        if(spv[(mindx_t)l_MxIndex].v.mc_fs_state == FS_STATE_FAULT)
        {
            func[(mindx_t)l_MxIndex].mcBrdPwmDisable();
            mpv[(mindx_t)l_MxIndex].v.pwm_enabled = MC_FALSE;
            mpv[(mindx_t)l_MxIndex].v.state = FAULT;
    }
    else
    {
            mcSlowLoopHandlerMx(&mpv[(mindx_t)l_MxIndex]);
        }
    }
    else
    {
        /* do nothing */
    }
}

void mcHandlerML(void)
{
    mindx_t i;
    
    for(i = 0; i < mc.motorIndex; i++)
    {
        mcMainLoopHandlerMx(&mpv[i]);
        fsHandlerML(&spv[i]);
    }
}

void mcSetSpeedControllerParameters(int32_t l_MxIndex, bool l_overwrite, float l_kp, float l_ki)
{
    if(l_overwrite == 0)
    {
        mpv[(mindx_t)l_MxIndex].v.piSpeedOverwrite = 0;
    }
    else
    {
        mpv[(mindx_t)l_MxIndex].v.piSpeedOverwrite = 1;
        mpv[(mindx_t)l_MxIndex].v.piSpeed.kp = (mcc_t)l_kp;
        mpv[(mindx_t)l_MxIndex].v.piSpeed.ki = (mcc_t)l_ki / (mcc_t)mpv[(mindx_t)l_MxIndex].p.freqSL;
    }
}

void mcSetSpeedControllerInteg(int32_t l_MxIndex, float l_sp_integ)
{
    mpv[(mindx_t)l_MxIndex].v.piSpeed.integ = l_sp_integ;
}

void mcBypassSpController(int32_t l_MxIndex, bool l_bypass, float l_iqref, float l_spref)
{
    if(l_bypass != MC_FALSE)
    {
        mpv[(mindx_t)l_MxIndex].v.bypass.spController = MC_TRUE;
        mpv[(mindx_t)l_MxIndex].v.idqref.q = l_iqref;
    }
    else if(mpv[(mindx_t)l_MxIndex].v.bypass.spController == MC_TRUE)
    {
        mpv[(mindx_t)l_MxIndex].v.idqref.q = l_iqref;
        mpv[(mindx_t)l_MxIndex].v.piSpeed.integ = mpv[(mindx_t)l_MxIndex].v.idqref.q;
        mpv[(mindx_t)l_MxIndex].v.spref = l_spref;
        mpv[(mindx_t)l_MxIndex].v.bypass.spController = MC_FALSE;
    }
    else
    {
        mpv[(mindx_t)l_MxIndex].v.bypass.spController = MC_FALSE;
    }
}

void mcEnableMotorControl(int32_t l_MxIndex)
{
    mpv[(mindx_t)l_MxIndex].control_state = MC_CONTROL_STATE_ACTIVE;
}

void mcDisableMotorControl(int32_t l_MxIndex)
{
    mpv[(mindx_t)l_MxIndex].control_state = MC_CONTROL_STATE_DISABLED;
    mpv[(mindx_t)l_MxIndex].v.sprefmec = (mcsp_t)0;
}

bool mcGetMotorControlState(int32_t l_MxIndex)
{
    if(mpv[(mindx_t)l_MxIndex].control_state == MC_CONTROL_STATE_DISABLED &&
    mpv[(mindx_t)l_MxIndex].v.spact == (mcsp_t)0)
    {
        mpv[(mindx_t)l_MxIndex].control_state = MC_CONTROL_STATE_PASSIVE;
    }

    return mpv[(mindx_t)l_MxIndex].control_state;
}

void mcActivateFunctionalsafety(int32_t l_MxIndex)
{
    spv[(mindx_t)l_MxIndex].enabled = MC_TRUE;
}

mc_float_t mcGetLd(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].p.phys.Ld;
}

mc_float_t mcGetLq(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].p.phys.Lq;
}

mc_float_t mcGetfsFatalFaultFlag(const int32_t l_MxIndex)
{
    return spv[(mindx_t)l_MxIndex].v.fsFatalFaultFlag;
}

mc_float_t mcGetStopRPMValue(const int32_t l_MxIndex)
{
    return mpv[(mindx_t)l_MxIndex].p.minsp_c2o;
}

uint32_t mcGetGeneratedFaultsBinary(int32_t l_MxIndex)
{
    return spv[(mindx_t)l_MxIndex].v.generatedFaultsBinary;
}

/************************ Function definitions - stop *************************/
