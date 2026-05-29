/**
*
*
*/


#include "mcinclude.h"

static void fsResetErrorCountersFlags(fsSpv_t *p_spv);
static void fsResetAllFaultCounters(fsSpv_t *p_spv);
static void fsDCBusVoltageCheck(fsSpv_t *p_spv);
static void fsDCBusVoltageIntegralCheck(fsSpv_t *p_spv);
static void fsIpmTempCheck(fsSpv_t *p_spv);
static void fsIpmFaultOutCheck(fsSpv_t *p_spv);
static void fsOverCurrentCheck(fsSpv_t *p_spv);
static void fsCurrentIntegralCheck(fsSpv_t *p_spv);
static void fsCurrentSamplingCheck(fsSpv_t *p_spv);
static void fsTargetCurrentCheck(fsSpv_t *p_spv);
static void fsLopCheck(fsSpv_t *p_spv);
static void TargetSpeedCheck(fsSpv_t *p_spv);
static void RefSpeedCheck(fsSpv_t *p_spv);
static void SpeedEstCheck(fsSpv_t *p_spv);
static void SpeedControllerCheck(fsSpv_t *p_spv);
static void OnTheFlyDurationCheck(fsSpv_t *p_spv);
static void StartupOpenLoopDurationCheck(fsSpv_t *p_spv);
static void TurnoffOpenLoopDurationCheck(fsSpv_t *p_spv);
static void fsOverLoadCheck(fsSpv_t *p_spv);
static void ThermalProtectionHandlerReset(fsSpv_t *p_spv);
static void ThermalProtectionHandlerSL(fsSpv_t *p_spv);
static void ThermalProtectionHandlerFL(fsSpv_t *p_spv);


/**
* @brief Functional safety layer initialization function
* 
* Function will load all initial values of parameters and variables about functional safety layer. All related 
* sublayers or process related to functional safety layer will be initialized in this function body.* 
*/
void fsInitFuncSafety(void)
{
    
}

/**
* @brief Functional safety layer initialization function
* 
* Function will load all initial values of parameters and variables about functional safety layer. All related 
* sublayers or process related to functional safety layer will be initialized in this function body.* 
*/
void fsAddMxFuncSafety(mcCpv_t *p_mpv, fsSpv_t *p_spv, const fsPar_t *p_fsp, mcMotorFunc_t *func)
{
    p_spv->v.mc_fs_state = FS_STATE_NO_FAULT;

    p_spv->v.mcvars.idqref = &p_mpv->v.idqref;
    p_spv->v.mcvars.idq = &p_mpv->v.idq;
    p_spv->v.mcvars.vdq = &p_mpv->v.vdq;
    p_spv->v.mcvars.thivec = &p_mpv->v.th_iref_vec;
    p_spv->v.mcvars.sprefin = &p_mpv->v.sprefin;
    p_spv->v.mcvars.spref = &p_mpv->v.spref;
    p_spv->v.mcvars.spest = &p_mpv->v.spest;
    p_spv->v.mcvars.state = &p_mpv->v.state;
    p_spv->v.mcvars.mc_state = &p_mpv->v.fs_state;
    
    p_spv->p = *p_fsp;
}

/**
* @brief functional safety layer fast loop handler function
* 
* Function will handle functional safety related functions in fast loop (Highest frequency handler)
*/
void fsFastLoopHandlerMx(fsSpv_t *p_spv)
{    
    if(p_spv->enabled == MC_FALSE)
    {
        return;
    }
    else if(p_spv->v.mc_fs_state != FS_STATE_FAULT)
    {
        fsOverCurrentCheck(p_spv);
        fsCurrentSamplingCheck(p_spv);
        fsTargetCurrentCheck(p_spv);
        fsLopCheck(p_spv);
        fsDCBusVoltageCheck(p_spv);
        fsIpmFaultOutCheck(p_spv);
        ThermalProtectionHandlerFL(p_spv);

    }
    else
    {
        ;   /* do nothing */
    }
    p_spv->fsHandlerCntFL++;
}

/**
* @brief functional safety layer slow loop handler function
* 
* Function will handle functional safety related functions in slow loop (Highest frequency handler)
*/
void fsSlowLoopHandlerMx(fsSpv_t *p_spv)
{
    if(p_spv->enabled == MC_FALSE)
    {
        fsResetErrorCountersFlags(p_spv);
    }
    else if(p_spv->v.mc_fs_state != FS_STATE_FAULT)
    {
        fsCurrentIntegralCheck(p_spv);
        fsDCBusVoltageIntegralCheck(p_spv);
        TargetSpeedCheck(p_spv);
        RefSpeedCheck(p_spv);
        SpeedEstCheck(p_spv);
        SpeedControllerCheck(p_spv);
        ThermalProtectionHandlerSL(p_spv);
        OnTheFlyDurationCheck(p_spv);
        StartupOpenLoopDurationCheck(p_spv);
        fsOverLoadCheck(p_spv);
        TurnoffOpenLoopDurationCheck(p_spv);
        fsIpmTempCheck(p_spv);
        /*PowerCheck(p_spv) */
        /*MotorTempCheck(p_spv) */
        
        /* check if(sprefin > VALUE) */
        /* check if(sprefin > 0 && spref > (sprefin + 10) || sprefin < 0 && spref < (sprefin - 10)) */
        /* check if(spref * sprefin < 0) */
        /* check if(sp_est * spref < 0) */
        /* check if(Abs(sp_est - spref) > VALUE) */
        
        /* check if(power > VALUE) */
        /* check if(WindTemp > VALUE) */

        p_spv->v.noFaultCntr++;
        if((p_spv->p.noFaultCntrMax != 0) && 
            (p_spv->v.noFaultCntr >= p_spv->p.noFaultCntrMax))
        {
            p_spv->v.noFaultCntr = (htck_t)0;
            fsResetAllFaultCounters(p_spv);
        }
    }
    else
    {
        fsResetErrorCountersFlags(p_spv);
        
        if(p_spv->v.fsFatalFaultFlag == MC_TRUE)
        {
            p_spv->v.halt_counter = 0u;
        }
        else
        {
            p_spv->v.halt_counter++;
            if(p_spv->v.halt_counter >= p_spv->v.halt_duration)
            {
                p_spv->v.mc_fs_state = FS_STATE_NO_FAULT;
                p_spv->v.halt_counter = 0;
            }
            else
            {
                ;   /* do nothing */
            }
        }
    }        
    p_spv->fsHandlerCntSL++;
}

/**
* @brief functional safety layer main loop handler function
* 
* Function will handle functional safety related functions in main loop
*/
void fsHandlerML(fsSpv_t *p_spv)
{
    ;   /* do nothing */
}

static void fsResetErrorCountersFlags(fsSpv_t *p_spv)
{
    p_spv->v.dcbus_voltage.flag = MC_FALSE;
    p_spv->v.dcbus_integral.flag = MC_FALSE;
    p_spv->v.over_current.flag = MC_FALSE;
    p_spv->v.current_integral.flag = MC_FALSE;
    p_spv->v.current_sampling.flag = MC_FALSE;
    p_spv->v.target_current.flag = MC_FALSE;
    p_spv->v.target_current.flagD = MC_FALSE;
    p_spv->v.target_current.flagQ = MC_FALSE;
    p_spv->v.lop.flag = MC_FALSE;
    p_spv->v.lop.flagPhU = MC_FALSE;
    p_spv->v.lop.flagPhV = MC_FALSE;
    p_spv->v.lop.flagPhW = MC_FALSE;
    p_spv->v.est_speed.flag = MC_FALSE;
    p_spv->v.sp_controller.flag = MC_FALSE;
    p_spv->v.ipm_temp.flag = MC_FALSE;
    p_spv->v.ipm_fault_out.flag = MC_FALSE;
    p_spv->v.ref_speed.flag = MC_FALSE;
    p_spv->v.onthefly_duration.flag = MC_FALSE;
    p_spv->v.startupol_duration.flag = MC_FALSE;
    p_spv->v.turnoffol_duration.flag = MC_FALSE;
    p_spv->v.over_load.flag = MC_FALSE;
    p_spv->v.thermalProtection.flag = MC_FALSE;
    
    p_spv->v.dcbus_voltage.maxErrorCntr = 0u;
    p_spv->v.dcbus_voltage.minErrorCntr = 0u;
    p_spv->v.over_current.phaseCurrentErrorCntr = 0u;
    p_spv->v.over_current.vector2CurrentErrorCntr = 0u;
    p_spv->v.current_sampling.errorCntr = 0u;
    p_spv->v.target_current.errorCntrD = 0u;
    p_spv->v.target_current.errorCntrQ = 0u;
    p_spv->v.lop.errorCntrPhU = 0u;
    p_spv->v.lop.errorCntrPhV = 0u;
    p_spv->v.lop.errorCntrPhW = 0u;
    p_spv->v.est_speed.errorCntr = 0u;
    p_spv->v.sp_controller.errorCntr = 0u;
    p_spv->v.onthefly_duration.cntr = 0;
    p_spv->v.startupol_duration.cntr = 0u;
    p_spv->v.turnoffol_duration.cntr = 0u;

    p_spv->v.current_integral.intg = (mcc_t)0;
    p_spv->v.current_integral.lowCurrentCntr = 0;
    p_spv->v.current_integral.windowCntr = 0;

    p_spv->v.dcbus_integral.intg = (mcv_t)0;
    p_spv->v.dcbus_integral.lowVoltageCntr = 0;
    p_spv->v.dcbus_integral.windowCntr = 0;

    p_spv->v.over_load.intg = 0;

    p_spv->v.noFaultCntr = (htck_t)0;
}

static void fsResetAllFaultCounters(fsSpv_t *p_spv)
{
    p_spv->v.dcbus_voltage.faultCntr = MC_FALSE;
    p_spv->v.dcbus_integral.faultCntr = MC_FALSE;
    p_spv->v.over_current.phaseCurrentFaultCntr = MC_FALSE;
    p_spv->v.over_current.vector2CurrentFaultCntr = MC_FALSE;
    p_spv->v.current_integral.faultCntr = MC_FALSE;
    p_spv->v.current_sampling.faultCntr = MC_FALSE;
    p_spv->v.target_current.faultCntrD = MC_FALSE;
    p_spv->v.target_current.faultCntrQ = MC_FALSE;
    p_spv->v.lop.faultCntrPhU = MC_FALSE;
    p_spv->v.lop.faultCntrPhV = MC_FALSE;
    p_spv->v.lop.faultCntrPhW = MC_FALSE;
    p_spv->v.est_speed.faultCntr = MC_FALSE;
    p_spv->v.sp_controller.faultCntr = MC_FALSE;
    p_spv->v.ipm_temp.faultCntr = MC_FALSE;
    p_spv->v.ipm_fault_out.faultCntr = MC_FALSE;
    p_spv->v.ref_speed.faultCntr = MC_FALSE;
    p_spv->v.onthefly_duration.faultCntr = MC_FALSE;
    p_spv->v.startupol_duration.faultCntr = MC_FALSE;
    p_spv->v.turnoffol_duration.faultCntr = MC_FALSE;
    p_spv->v.over_load.faultCntr = MC_FALSE;
    p_spv->v.thermalProtection.faultCntr = MC_FALSE;
}

static void fsDCBusVoltageCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.dcbus_voltage.enabled == MC_FALSE)
    {
        p_spv->v.dcbus_voltage.maxErrorCntr = 0u;
        p_spv->v.dcbus_voltage.minErrorCntr = 0u;
        p_spv->v.dcbus_voltage.faultCntr = 0u;
        
        return;
    }
    else
    {
    if(p_spv->v.vbus_pin_circuit_state)
    {
      p_spv->v.dcbus_voltage.openErrorCntr++;
      if(p_spv->v.dcbus_voltage.openErrorCntr >= p_spv->p.dcbus_voltage.errorCntrPinOpenMax)
      {
        p_spv->v.halt_duration = p_spv->p.dcbus_voltage.duration;
        p_spv->v.dcbus_voltage.flag = MC_TRUE;
        
        p_spv->v.generatedFaultsBinary &= 0x00;
        p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_DCBUS;
        
        *p_spv->v.mcvars.state = FAULT;
        p_spv->v.fsTotalGeneratedFault++;
        p_spv->v.dcbus_voltage.faultCntr++;
        if(p_spv->v.dcbus_voltage.faultCntr >= p_spv->p.dcbus_voltage.faultCntrMax)
        {
          p_spv->v.fsFatalFaultFlag = MC_TRUE;
          p_spv->v.dcbus_voltage.faultCntr = p_spv->p.dcbus_voltage.faultCntrMax;
        }
        else
        {
          /* do nothing */
        }
        p_spv->v.dcbus_voltage.maxErrorCntr = p_spv->p.dcbus_voltage.errorCntrMax;
      }
      else
      {
        /* do nothing */
      }
    }
        if(p_spv->v.vbus > p_spv->p.dcbus_voltage.maxDcBus)
        {
            p_spv->v.dcbus_voltage.maxErrorCntr++;
            if(p_spv->v.dcbus_voltage.maxErrorCntr >= p_spv->p.dcbus_voltage.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.dcbus_voltage.duration;
                p_spv->v.dcbus_voltage.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_DCBUS;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
            
                p_spv->v.dcbus_voltage.faultCntr++;
                if(p_spv->p.dcbus_voltage.faultCntrMax != 0 && p_spv->v.dcbus_voltage.faultCntr >= p_spv->p.dcbus_voltage.faultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.dcbus_voltage.faultCntr = p_spv->p.dcbus_voltage.faultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.dcbus_voltage.maxErrorCntr = p_spv->p.dcbus_voltage.errorCntrMax;
            }
            else
            {
                /* do nothing */
            }
        }
        else if(p_spv->v.dcbus_voltage.maxErrorCntr > 0u)
        {
            p_spv->v.dcbus_voltage.maxErrorCntr--;
        }
        else
        {
            /* do nothing */
        }
        
        if(p_spv->v.vbus < p_spv->p.dcbus_voltage.minDcBus)
        {
            p_spv->v.dcbus_voltage.minErrorCntr++;
            if(p_spv->v.dcbus_voltage.minErrorCntr >= p_spv->p.dcbus_voltage.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.dcbus_voltage.duration;
                p_spv->v.dcbus_voltage.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_DCBUS;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
                
                p_spv->v.dcbus_voltage.faultCntr++;
                if(p_spv->p.dcbus_voltage.faultCntrMax != 0 && p_spv->v.dcbus_voltage.faultCntr >= p_spv->p.dcbus_voltage.faultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.dcbus_voltage.faultCntr = p_spv->p.dcbus_voltage.faultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.dcbus_voltage.minErrorCntr = p_spv->p.dcbus_voltage.errorCntrMax;
            }
            else
            {
                /* do nothing */
            }
        }
        else if(p_spv->v.dcbus_voltage.minErrorCntr > 0u)
        {
            p_spv->v.dcbus_voltage.minErrorCntr--;
        }        
        else
        {
            /* do nothing */
        }
    }
}

static void fsDCBusVoltageIntegralCheck(fsSpv_t *p_spv)
{
    mcc_t _vbus, _vbus_ave;

    if(p_spv->p.dcbus_integral.enabled == MC_FALSE)
    {
        p_spv->v.dcbus_integral.faultCntr = 0u;
        
        return;
    }
    else
    {    
         _vbus = p_spv->v.vbus;

         if(_vbus > p_spv->p.dcbus_integral.minVoltage)
         {
            p_spv->v.dcbus_integral.lowVoltageCntr = 0;
            p_spv->v.dcbus_integral.intg += _vbus - p_spv->p.dcbus_integral.minVoltage;
            if(p_spv->v.dcbus_integral.intg < 0.0f)
                p_spv->v.dcbus_integral.intg = 0.0f;

            p_spv->v.dcbus_integral.windowCntr++;
            if(p_spv->v.dcbus_integral.windowCntr >= p_spv->p.dcbus_integral.windowCntrMax)
            {
                p_spv->v.dcbus_integral.windowCntr = p_spv->p.dcbus_integral.windowCntrMax;

                if(p_spv->v.dcbus_integral.intg > p_spv->p.dcbus_integral.intgMax)
                {
                    p_spv->v.halt_duration = p_spv->p.dcbus_integral.duration;
                    p_spv->v.dcbus_integral.flag = MC_TRUE;
                        
                    p_spv->v.generatedFaultsBinary &= 0x00;
                    p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_DCBUS_INTEGRAL;
                        
                    p_spv->v.mc_fs_state = FS_STATE_FAULT;
                    p_spv->v.fsTotalGeneratedFault++;                
                        
                    p_spv->v.dcbus_integral.faultCntr++;
                    if(p_spv->p.dcbus_integral.faultCntrMax != 0 && p_spv->v.dcbus_integral.faultCntr >= p_spv->p.dcbus_integral.faultCntrMax)
                    {
                        p_spv->v.fsFatalFaultFlag = MC_TRUE;
                        p_spv->v.dcbus_integral.faultCntr = p_spv->p.dcbus_integral.faultCntrMax;
                    }
                    else
                    {
                        /* do nothing */
                    }
                       
                    return;
                }
                
                _vbus_ave = p_spv->v.dcbus_integral.intg / p_spv->v.dcbus_integral.windowCntr;
                p_spv->v.dcbus_integral.intg = p_spv->v.dcbus_integral.intg - _vbus_ave;
            }
        }
        else
        {
            p_spv->v.dcbus_integral.lowVoltageCntr++;

            if(p_spv->v.dcbus_integral.lowVoltageCntr >= p_spv->p.dcbus_integral.lowVoltageCntrMax)
            {
                p_spv->v.dcbus_integral.lowVoltageCntr = p_spv->p.dcbus_integral.lowVoltageCntrMax;
                p_spv->v.dcbus_integral.windowCntr = 0;
                p_spv->v.dcbus_integral.intg = (mcv_t)0;
            }
        }
    }
}

static void fsIpmTempCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.ipm_temp.enabled == MC_FALSE)
    {
        p_spv->v.ipm_temp.faultCntr = 0u;
        
        return;
    }
    else if(p_spv->v.ipm_temp_sampled > p_spv->p.ipm_temp.maxTemp)
    {
        p_spv->v.halt_duration = p_spv->p.ipm_temp.duration;
        p_spv->v.ipm_temp.flag = MC_TRUE;
        
        p_spv->v.generatedFaultsBinary &= 0x00;
        p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_IPM_TEMP;
        
        p_spv->v.mc_fs_state = FS_STATE_FAULT;
        p_spv->v.fsTotalGeneratedFault++;
        
        p_spv->v.ipm_temp.faultCntr++;
        if(p_spv->p.ipm_temp.faultCntrMax != 0 && p_spv->v.ipm_temp.faultCntr >= p_spv->p.ipm_temp.faultCntrMax)
        {
            p_spv->v.ipm_temp.faultCntr = p_spv->p.ipm_temp.faultCntrMax;
            p_spv->v.fsFatalFaultFlag = MC_TRUE;
        }
        else
        {
            /* do nothing */
        }
    }
    else
    {
        /* do nothing */
    }
}

static void fsIpmFaultOutCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.ipm_fault_out.enabled == MC_FALSE)
    {
        p_spv->v.ipm_fault_out.faultCntr = 0u;
        
        return;
    }
    else if(p_spv->v.ipm_faultout == MC_TRUE)
    {
        p_spv->v.halt_duration = p_spv->p.ipm_fault_out.duration;
        p_spv->v.ipm_fault_out.flag = MC_TRUE;
        
        p_spv->v.generatedFaultsBinary &= 0x00;
        p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_IPM_FAULT;
        
        p_spv->v.mc_fs_state = FS_STATE_FAULT;
        p_spv->v.fsTotalGeneratedFault++;
        
        p_spv->v.ipm_fault_out.faultCntr++;
        if(p_spv->p.ipm_fault_out.faultCntrMax != 0 && p_spv->v.ipm_fault_out.faultCntr >= p_spv->p.ipm_fault_out.faultCntrMax)
        {
            p_spv->v.ipm_fault_out.faultCntr = p_spv->p.ipm_fault_out.faultCntrMax;
            p_spv->v.fsFatalFaultFlag = MC_TRUE;
        }
        else
        {
            /* do nothing */
        }
    }
    else
    {
        /* do nothing */
    }
}

static void fsOverCurrentCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.over_current.enabled == MC_FALSE)
    {
        p_spv->v.over_current.phaseCurrentErrorCntr = 0u;
        p_spv->v.over_current.phaseCurrentFaultCntr = 0u;
        
        p_spv->v.over_current.vector2CurrentErrorCntr = 0u;
        p_spv->v.over_current.vector2CurrentFaultCntr = 0u;
        
        return;
    }
    else
    {    
        if((p_spv->v.iuvw_abs.u > p_spv->p.over_current.maxPhaseCurrent) || 
           (p_spv->v.iuvw_abs.v > p_spv->p.over_current.maxPhaseCurrent) || 
               (p_spv->v.iuvw_abs.w > p_spv->p.over_current.maxPhaseCurrent))
        {
            p_spv->v.over_current.phaseCurrentErrorCntr++;
            if(p_spv->v.over_current.phaseCurrentErrorCntr > 
               p_spv->p.over_current.phaseCurrentErrorCntrsMax)
            {
                p_spv->v.halt_duration = p_spv->p.over_current.duration;
                p_spv->v.over_current.faultFlagPhaseCurrent = MC_TRUE;
                p_spv->v.over_current.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_OVER_CURRENT;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;                
                
                p_spv->v.over_current.phaseCurrentFaultCntr++;
                if(p_spv->v.over_current.phaseCurrentFaultCntr >= 
                   p_spv->p.over_current.phaseCurrentFaultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.over_current.phaseCurrentFaultCntr = p_spv->p.over_current.phaseCurrentFaultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.over_current.phaseCurrentErrorCntr = p_spv->p.over_current.phaseCurrentErrorCntrsMax;
                
                return;
            }
        }
        else if(p_spv->v.over_current.phaseCurrentErrorCntr > 0u)
        {
            p_spv->v.over_current.phaseCurrentErrorCntr--;
        }
        else
        {
            /* do nothing */
        }
        
        if(p_spv->v.iamp2 > p_spv->p.over_current.maxVector2Current)
        {
            p_spv->v.over_current.vector2CurrentErrorCntr++;
            if(p_spv->v.over_current.vector2CurrentErrorCntr > 
               p_spv->p.over_current.vector2CurrentErrorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.over_current.duration;
                p_spv->v.over_current.faultFlagVectorCurrent = MC_TRUE;
                p_spv->v.over_current.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_OVER_CURRENT;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;                
                
                p_spv->v.over_current.vector2CurrentFaultCntr++;
                if(p_spv->v.over_current.vector2CurrentFaultCntr >= 
                   p_spv->p.over_current.vector2CurrentFaultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.over_current.vector2CurrentFaultCntr = p_spv->p.over_current.vector2CurrentFaultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.over_current.phaseCurrentErrorCntr = p_spv->p.over_current.phaseCurrentErrorCntrsMax;
                
                return;
            }
            else
            {
                /* do nothing */
            }
        }
        else if(p_spv->v.over_current.vector2CurrentErrorCntr > 0u)
        {
            p_spv->v.over_current.vector2CurrentErrorCntr--;
        }
        else
        {
            /* do nothing */
        }
    }
}

static void fsCurrentIntegralCheck(fsSpv_t *p_spv)
{
    mcc_t _iamp2, _iamp2_ave;

    if(p_spv->p.current_integral.enabled == MC_FALSE)
    {
        p_spv->v.current_integral.faultCntr = 0u;
        
        return;
    }
    else
    {    
         _iamp2 = p_spv->v.iamp2;

         if(_iamp2 > p_spv->p.current_integral.minCurrent)
         {
            p_spv->v.current_integral.lowCurrentCntr = 0;
            p_spv->v.current_integral.intg += _iamp2 - p_spv->p.current_integral.minCurrent;
            if(p_spv->v.current_integral.intg < 0.0f)
                p_spv->v.current_integral.intg = 0.0f;

            p_spv->v.current_integral.windowCntr++;
            if(p_spv->v.current_integral.windowCntr >= p_spv->p.current_integral.windowCntrMax)
            {
                p_spv->v.current_integral.windowCntr = p_spv->p.current_integral.windowCntrMax;

                if(p_spv->v.current_integral.intg > p_spv->p.current_integral.intgMax)
                {
                    p_spv->v.halt_duration = p_spv->p.current_integral.duration;
                    p_spv->v.current_integral.flag = MC_TRUE;
                        
                    p_spv->v.generatedFaultsBinary &= 0x00;
                    p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_CURRENT_INTEGRAL;
                        
                    p_spv->v.mc_fs_state = FS_STATE_FAULT;
                    p_spv->v.fsTotalGeneratedFault++;                
                        
                    p_spv->v.current_integral.faultCntr++;
                    if(p_spv->p.current_integral.faultCntrMax != 0 && p_spv->v.current_integral.faultCntr >= p_spv->p.current_integral.faultCntrMax)
                    {
                        p_spv->v.fsFatalFaultFlag = MC_TRUE;
                        p_spv->v.current_integral.faultCntr = p_spv->p.current_integral.faultCntrMax;
                    }
                    else
                    {
                        /* do nothing */
                    }
                       
                    return;
                }
                
                _iamp2_ave = p_spv->v.current_integral.intg / p_spv->v.current_integral.windowCntr;
                p_spv->v.current_integral.intg = p_spv->v.current_integral.intg - _iamp2_ave;
            }
        }
        else
        {
            p_spv->v.current_integral.lowCurrentCntr++;

            if(p_spv->v.current_integral.lowCurrentCntr >= p_spv->p.current_integral.lowCurrentCntrMax)
            {
                p_spv->v.current_integral.lowCurrentCntr = p_spv->p.current_integral.lowCurrentCntrMax;
                p_spv->v.current_integral.windowCntr = 0;
                p_spv->v.current_integral.intg = (mcc_t)0;
            }
        }
    }
}

static void fsCurrentSamplingCheck(fsSpv_t *p_spv)
{
    mcuvw_t _iuvw;
    
    if(p_spv->p.current_sampling.enabled == MC_FALSE)
    {
        p_spv->v.current_sampling.errorCntr = 0u;
        p_spv->v.current_sampling.faultCntr = 0u;
        
        p_spv->v.current_sampling.flag = MC_FALSE;
        
        return;
    }
    else
    {
        _iuvw = p_spv->v.iuvw;

        p_spv->v.current_sampling.sumOfPhCurrents = _iuvw.u + _iuvw.v + _iuvw.w;

        if(p_spv->v.current_sampling.sumOfPhCurrents > p_spv->p.current_sampling.sumOfPhCurrentsMax)
        {
            p_spv->v.current_sampling.errorCntr++;
            if(p_spv->v.current_sampling.errorCntr > p_spv->p.current_sampling.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.current_sampling.duration;
                p_spv->v.current_sampling.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_CURRENT_SAMPLING;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
                
                p_spv->v.current_sampling.faultCntr++;
                if(p_spv->p.current_sampling.faultCntrMax != 0 && p_spv->v.current_sampling.faultCntr >= p_spv->p.current_sampling.faultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.current_sampling.faultCntr = p_spv->p.current_sampling.faultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.current_sampling.errorCntr = p_spv->p.current_sampling.errorCntrMax;
                
                return;
            }
            else
            {
                /* do nothing */
            }
        }
        else if(p_spv->v.current_sampling.errorCntr > 0u)
        {
            p_spv->v.current_sampling.errorCntr--;
        }
        else
        {
            /* do nothing */
        }
    }
}

static void fsTargetCurrentCheck(fsSpv_t *p_spv)
{
    mcdq_t idqref;
    mcdq_t idq;
    
    if(p_spv->p.target_current.enabled == MC_FALSE)
    {
        p_spv->v.target_current.errorCntrD = 0u;
        p_spv->v.target_current.errorCntrQ = 0u;
        
        p_spv->v.target_current.faultCntrD = 0u;
        p_spv->v.target_current.faultCntrQ = 0u;
        
        p_spv->v.target_current.flag = MC_FALSE;

        return;
    }
    else
    {    
        idqref = *p_spv->v.mcvars.idqref;
        idq = *p_spv->v.mcvars.idq;
        
        p_spv->v.target_current.errCurrentD = idq.d - idqref.d;
        
        if(p_spv->v.target_current.errCurrentD > 
           p_spv->p.target_current.errCurrentMaxD)
        {
            p_spv->v.target_current.errorCntrD++;
            if(p_spv->v.target_current.errorCntrD > 
               p_spv->p.target_current.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.target_current.duration;
                p_spv->v.target_current.flagD = MC_TRUE;
                p_spv->v.target_current.flag = MC_TRUE;

                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_TARGET_CURRENT;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
                
                p_spv->v.target_current.faultCntrD++;
                if(p_spv->p.target_current.faultCntrMax != 0 && 
                    p_spv->v.target_current.faultCntrD >= p_spv->p.target_current.faultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.target_current.faultCntrD = p_spv->p.current_sampling.faultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.target_current.errorCntrD = p_spv->p.target_current.errorCntrMax;
                
                return;
            }
            else
            {
                /* do nothing */
            }
        }
        else if(p_spv->v.target_current.errorCntrD > 0u)
        {
            p_spv->v.target_current.errorCntrD--;
        }
        else
        {
            /* do nothing */
        }
        
        p_spv->v.target_current.errCurrentQ = idq.q - idqref.q;
        
        if(p_spv->v.target_current.errCurrentQ > 
           p_spv->p.target_current.errCurrentMaxQ)
        {
            p_spv->v.target_current.errorCntrQ++;
            if(p_spv->v.target_current.errorCntrQ > 
               p_spv->p.target_current.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.target_current.duration;
                p_spv->v.target_current.flagQ = MC_TRUE;
                p_spv->v.target_current.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_TARGET_CURRENT;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
                
                p_spv->v.target_current.faultCntrQ++;
                if(p_spv->p.target_current.faultCntrMax != 0 && 
                    p_spv->v.target_current.faultCntrQ >= p_spv->p.target_current.faultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.target_current.faultCntrQ = p_spv->p.target_current.faultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.target_current.errorCntrQ = p_spv->p.target_current.errorCntrMax;
                
                return;
            }
            else
            {
                /* do nothing */
            }
        }
        else if(p_spv->v.target_current.errorCntrQ > 0u)
        {
            p_spv->v.target_current.errorCntrQ--;
        }
        else
        {
            /* do nothing */
        }
    }
}

static void fsLopCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.lop.enabled == MC_FALSE)
    {
        p_spv->v.lop.errorCntrPhU = 0u;        
        p_spv->v.lop.errorCntrPhV = 0u;        
        p_spv->v.lop.errorCntrPhW = 0u;
        
        p_spv->v.lop.flag = MC_FALSE;
        
        return;
    }
    else if(p_spv->v.iamp < p_spv->p.lop.iamp_min)
    {
        if(p_spv->v.lop.errorCntrPhU > 0u)
            p_spv->v.lop.errorCntrPhU--;        
        
        if(p_spv->v.lop.errorCntrPhV > 0u)
            p_spv->v.lop.errorCntrPhV--;        
        
        if(p_spv->v.lop.errorCntrPhW > 0u)
            p_spv->v.lop.errorCntrPhW--;        

        return;
    }
    else
    {
        p_spv->v.lop.iuvwMin = p_spv->v.iamp * p_spv->p.lop.threCoeff;
        p_spv->v.lop.th = *p_spv->v.mcvars.thivec;
        
        if((p_spv->p.lop.check_th == MC_TRUE) && 
            (((p_spv->v.lop.th >= p_spv->p.lop.NoCurrentPos1PhUmin) && (p_spv->v.lop.th <= p_spv->p.lop.NoCurrentPos1PhUmax)) ||
             ((p_spv->v.lop.th >= p_spv->p.lop.NoCurrentPos2PhUmin) && (p_spv->v.lop.th <= p_spv->p.lop.NoCurrentPos2PhUmax))))
        {
            /* do nothing */
        }
        else
        {
            if(p_spv->v.iuvw_abs.u < p_spv->v.lop.iuvwMin)
            {
                p_spv->v.lop.errorCntrPhU++;
                if(p_spv->v.lop.errorCntrPhU > p_spv->p.lop.errorCntrMax)
                {
                    p_spv->v.halt_duration = p_spv->p.lop.duration;
                    p_spv->v.lop.flagPhU = MC_TRUE;
                    p_spv->v.lop.flag = MC_TRUE;
                    
                    p_spv->v.generatedFaultsBinary &= 0x00;
                    p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_LOP;
                    
                    p_spv->v.mc_fs_state = FS_STATE_FAULT;
                    p_spv->v.fsTotalGeneratedFault++;
                    
                    p_spv->v.lop.faultCntrPhU++;
                    if(p_spv->p.lop.faultCntrMax != 0 && p_spv->v.lop.faultCntrPhU >= p_spv->p.lop.faultCntrMax)
                    {
                        p_spv->v.fsFatalFaultFlag = MC_TRUE;
                        p_spv->v.lop.faultCntrPhU = p_spv->p.lop.faultCntrMax;
                    }
                    else
                    {
                        /* do nothing */
                    }
                    p_spv->v.lop.errorCntrPhU = p_spv->p.lop.errorCntrMax;
                    
                    return;
                }
                else
                {
                    /* do nothing */
                }
            }
            else if(p_spv->v.lop.errorCntrPhU > 0u)
            {
                p_spv->v.lop.errorCntrPhU--;
            }
            else
            {
                /* do nothing */
            }
        }
        
        if((p_spv->p.lop.check_th == MC_TRUE) && 
            (((p_spv->v.lop.th >= p_spv->p.lop.NoCurrentPos1PhVmin) && (p_spv->v.lop.th <= p_spv->p.lop.NoCurrentPos1PhVmax)) ||
             ((p_spv->v.lop.th >= p_spv->p.lop.NoCurrentPos2PhVmin) && (p_spv->v.lop.th <= p_spv->p.lop.NoCurrentPos2PhVmax))))
        {
            /* do nothing */
        }
        else
        {
            if(p_spv->v.iuvw_abs.v < p_spv->v.lop.iuvwMin)
            {
                p_spv->v.lop.errorCntrPhV++;
                if(p_spv->v.lop.errorCntrPhV > p_spv->p.lop.errorCntrMax)
                {
                    p_spv->v.halt_duration = p_spv->p.lop.duration;
                    p_spv->v.lop.flagPhV = MC_TRUE;
                    p_spv->v.lop.flag = MC_TRUE;
                    
                    p_spv->v.generatedFaultsBinary &= 0x00;
                    p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_LOP;
                    
                    p_spv->v.mc_fs_state = FS_STATE_FAULT;
                    p_spv->v.fsTotalGeneratedFault++;
                    
                    p_spv->v.lop.faultCntrPhV++;
                    if(p_spv->p.lop.faultCntrMax != 0 && p_spv->v.lop.faultCntrPhV >= p_spv->p.lop.faultCntrMax)
                    {
                        p_spv->v.fsFatalFaultFlag = MC_TRUE;
                        p_spv->v.lop.faultCntrPhV = p_spv->p.lop.faultCntrMax;
                    }
                    else
                    {
                        /* do nothing */
                    }
                    p_spv->v.lop.errorCntrPhV = p_spv->p.lop.errorCntrMax;
                    
                    return;
                }
                else
                {
                    /* do nothing */
                }
            }
            else if(p_spv->v.lop.errorCntrPhV > 0u)
            {
                p_spv->v.lop.errorCntrPhV--;
            }
            else
            {
                /* do nothing */
            }
        }
        
        if((p_spv->p.lop.check_th == MC_TRUE) && 
            (((p_spv->v.lop.th >= p_spv->p.lop.NoCurrentPos1PhWmin) && (p_spv->v.lop.th <= p_spv->p.lop.NoCurrentPos1PhWmax)) ||
             ((p_spv->v.lop.th >= p_spv->p.lop.NoCurrentPos2PhWmin) && (p_spv->v.lop.th <= p_spv->p.lop.NoCurrentPos2PhWmax))))
        {
            /* do nothing */
        }
        else
        {
            if(p_spv->v.iuvw_abs.w < p_spv->v.lop.iuvwMin)
            {
                p_spv->v.lop.errorCntrPhW++;
                if(p_spv->v.lop.errorCntrPhW > p_spv->p.lop.errorCntrMax)
                {
                    p_spv->v.halt_duration = p_spv->p.lop.duration;
                    p_spv->v.lop.flagPhW = MC_TRUE;
                    p_spv->v.lop.flag = MC_TRUE;
                    
                    p_spv->v.generatedFaultsBinary &= 0x00;
                    p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_LOP;
                    
                    p_spv->v.mc_fs_state = FS_STATE_FAULT;
                    p_spv->v.fsTotalGeneratedFault++;
                    
                    p_spv->v.lop.faultCntrPhW++;
                    if(p_spv->p.lop.faultCntrMax != 0 && p_spv->v.lop.faultCntrPhW >= p_spv->p.lop.faultCntrMax)
                    {
                        p_spv->v.fsFatalFaultFlag = MC_TRUE;
                        p_spv->v.lop.faultCntrPhW = p_spv->p.lop.faultCntrMax;
                    }
                    else
                    {
                        /* do nothing */
                    }
                    p_spv->v.lop.errorCntrPhW = p_spv->p.lop.errorCntrMax;
                    
                    return;
                }
                else
                {
                    /* do nothing */
                }
            }
            else if(p_spv->v.lop.errorCntrPhW > 0u)
            {
                p_spv->v.lop.errorCntrPhW--;
            }
            else
            {
                /* do nothing */
            }
        }
    }
}

static void TargetSpeedCheck(fsSpv_t *p_spv)
{
    mcsp_t l_sprefin;
    
    if(p_spv->p.target_speed.enabled == MC_FALSE)
    {
        p_spv->v.target_speed.flag = MC_FALSE;
        
        return;
    }
    else
    {
        l_sprefin = *p_spv->v.mcvars.sprefin;
        
        if(l_sprefin > (mcsp_t)p_spv->p.target_speed.maxSpeed)
        {
            p_spv->v.halt_duration = p_spv->p.target_speed.duration;
            p_spv->v.target_speed.flag = MC_TRUE;
            
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_TARGET_SPEED;
            
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;

            p_spv->v.target_speed.faultCntr++;
            if(p_spv->p.target_speed.faultCntrMax != 0 && p_spv->v.target_speed.faultCntr >= p_spv->p.target_speed.faultCntrMax)
            {
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
                p_spv->v.target_speed.faultCntr = p_spv->p.target_speed.faultCntrMax;
            }
            else
            {
                /* do nothing */
            }
            
            return;
        }
        else
        {
            /* do nothing */
        }
    }
}

static void RefSpeedCheck(fsSpv_t *p_spv)
{
    mcsp_t l_sprefin;
    mcsp_t l_spref;
    
    if(p_spv->p.ref_speed.enabled == MC_FALSE)
    {
        p_spv->v.ref_speed.flag = MC_FALSE;
        
        return;
    }
    else
    {
        l_sprefin = *p_spv->v.mcvars.sprefin;
        l_spref = *p_spv->v.mcvars.spref;
        
        if(((l_spref > (mcsp_t)0) && (l_spref > (mcsp_t)p_spv->p.ref_speed.maxSpeed)) || 
           ((l_spref < (mcsp_t)0) && (l_spref < -(mcsp_t)p_spv->p.ref_speed.maxSpeed)) ||
           ((l_spref * l_sprefin) < (mcsp_t)0))
        {
            p_spv->v.halt_duration = p_spv->p.ref_speed.duration;
            p_spv->v.ref_speed.flag = MC_TRUE;
            
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_REF_SPEED;
            
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;
            
            p_spv->v.ref_speed.faultCntr++;
            if(p_spv->p.ref_speed.faultCntrMax != 0 && p_spv->v.ref_speed.faultCntr >= p_spv->p.ref_speed.faultCntrMax)
            {
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
                p_spv->v.ref_speed.faultCntr = p_spv->p.ref_speed.faultCntrMax;
            }
            else
            {
                /* do nothing */
            }
            
            return;
        }
        else
        {
            /* do nothing */
        }
    }
}

static void SpeedEstCheck(fsSpv_t *p_spv)
{
    mcsp_t l_spref;
    mcsp_t l_spest;
    
    if(p_spv->p.est_speed.enabled == MC_FALSE)
    {
        p_spv->v.est_speed.errorCntr = 0u;
        p_spv->v.est_speed.flag = MC_FALSE;
        
        return;
    }
    else
    {
        l_spref = *p_spv->v.mcvars.spref;
        l_spest = *p_spv->v.mcvars.spest;
        
        if((l_spref * l_spest) < (mcsp_t)0)
        {
            p_spv->v.est_speed.errorCntr++;
            if(p_spv->v.est_speed.errorCntr > p_spv->p.est_speed.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.est_speed.duration;
                p_spv->v.est_speed.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_EST_SPEED;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
                
                p_spv->v.est_speed.faultCntr++;
                if(p_spv->p.est_speed.faultCntrMax != 0 && p_spv->v.est_speed.faultCntr >= p_spv->p.est_speed.faultCntrMax)
                {
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                    p_spv->v.est_speed.faultCntr = p_spv->p.est_speed.faultCntrMax;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.est_speed.errorCntr = p_spv->p.est_speed.errorCntrMax;
                
                return;
            }
            else
            {
                /* do nothing */
            }
        }
        else
        {
            if(p_spv->v.est_speed.errorCntr > 0u)
            {
                p_spv->v.est_speed.errorCntr--;
            }
            else
            {
                /* do nothing */
            }
        }
    }
}

static void SpeedControllerCheck(fsSpv_t *p_spv)
{
    mcsp_t l_spref;
    mcsp_t l_spest;
    mcsp_t l_sp_err;
    
    if(p_spv->p.sp_controller.enabled == MC_FALSE)
    {
        p_spv->v.sp_controller.errorCntr = 0u;
        p_spv->v.sp_controller.flag = MC_FALSE;
        
        return;
    }
    else if(*p_spv->v.mcvars.state != RAMP)
    {
        p_spv->v.sp_controller.errorCntr = 0u;        
    }
    else
    {    
        l_spref = *p_spv->v.mcvars.spref;
        l_spest = *p_spv->v.mcvars.spest;
        l_sp_err = l_spref - l_spest;
        
        if((l_sp_err > p_spv->p.sp_controller.speedErrorMax) || (l_sp_err < -p_spv->p.sp_controller.speedErrorMax))
        {
            p_spv->v.sp_controller.errorCntr++;
            if(p_spv->v.sp_controller.errorCntr > p_spv->p.sp_controller.errorCntrMax)
            {
                p_spv->v.halt_duration = p_spv->p.sp_controller.duration;
                p_spv->v.sp_controller.flag = MC_TRUE;
                
                p_spv->v.generatedFaultsBinary &= 0x00;
                p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_SP_CONTROLLER;
                
                p_spv->v.mc_fs_state = FS_STATE_FAULT;
                p_spv->v.fsTotalGeneratedFault++;
                
                p_spv->v.sp_controller.faultCntr++;
                if(p_spv->p.sp_controller.faultCntrMax != 0 && p_spv->v.sp_controller.faultCntr >= p_spv->p.sp_controller.faultCntrMax)
                {
                    p_spv->v.sp_controller.faultCntr = p_spv->p.sp_controller.faultCntrMax;
                    p_spv->v.fsFatalFaultFlag = MC_TRUE;
                }
                else
                {
                    /* do nothing */
                }
                p_spv->v.sp_controller.errorCntr = p_spv->p.sp_controller.errorCntrMax;
                
                return;
            }
            else
            {
                /* do nothing */
            }
        }
        else
        {
            if(p_spv->v.sp_controller.errorCntr > 0u)
            {
                p_spv->v.sp_controller.errorCntr--;
            }
            else
            {
                /* do nothing */
            }
        }
    }
}

static void OnTheFlyDurationCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.onthefly_duration.enabled == MC_FALSE)
    {
        p_spv->v.onthefly_duration.flag = MC_FALSE;
        
        return;
    }
    else if(*p_spv->v.mcvars.state != ON_THE_FLY)
    {
        p_spv->v.onthefly_duration.cntr = 0;
    }
    else
    {
        if(p_spv->v.onthefly_duration.cntr < p_spv->p.onthefly_duration.maxStateDuration)
        {
            p_spv->v.onthefly_duration.cntr++;
        }
        else
        {
            p_spv->v.onthefly_duration.flag = MC_TRUE;
            p_spv->v.halt_duration = p_spv->p.onthefly_duration.faultDuration;
            
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_ONTHEFLY_DURATION;
            
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;
            
            if(p_spv->p.onthefly_duration.faultCntrMax != 0 && p_spv->v.onthefly_duration.faultCntr < p_spv->p.onthefly_duration.faultCntrMax)
            {
                p_spv->v.onthefly_duration.faultCntr++;
            }
            else
            {
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
            }
        }
    }

}

static void StartupOpenLoopDurationCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.startupol_duration.enabled == MC_FALSE)
    {
        p_spv->v.startupol_duration.flag = MC_FALSE;
        
        return;
    }
    else if(*p_spv->v.mcvars.state != STARTUP)
    {
        p_spv->v.startupol_duration.cntr = 0;
    }
    else
    {
        p_spv->v.startupol_duration.cntr++;
        if(p_spv->v.startupol_duration.cntr > p_spv->p.startupol_duration.maxDuration)
        {
            p_spv->v.startupol_duration.flag = MC_TRUE;
            p_spv->v.halt_duration = p_spv->p.startupol_duration.duration;
            
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_STARTUPOL_DURATION;
            
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;
            
            p_spv->v.startupol_duration.faultCntr++;
            if(p_spv->p.startupol_duration.faultCntrMax != 0 && p_spv->v.startupol_duration.faultCntr >= p_spv->p.startupol_duration.faultCntrMax)
            {
                p_spv->v.startupol_duration.faultCntr = p_spv->p.startupol_duration.faultCntrMax;
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
            }
            else
            {
                /* do nothing */
            }
        }
    }
}

static void TurnoffOpenLoopDurationCheck(fsSpv_t *p_spv)
{
    if(p_spv->p.turnoffol_duration.enabled == MC_FALSE)
    {
        p_spv->v.turnoffol_duration.flag = MC_FALSE;
        
        return;
    }
    else if(*p_spv->v.mcvars.state != TURNOFF)
    {
        p_spv->v.turnoffol_duration.cntr = 0;
    }
    else
    {
        p_spv->v.turnoffol_duration.cntr++;
        if(p_spv->v.turnoffol_duration.cntr > p_spv->p.turnoffol_duration.maxDuration)
        {
            p_spv->v.turnoffol_duration.flag = MC_TRUE;
            p_spv->v.halt_duration = p_spv->p.turnoffol_duration.duration;
            
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_TURNOFFOL_DURATION;
            
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;
            
            p_spv->v.turnoffol_duration.faultCntr++;
            if(p_spv->p.turnoffol_duration.faultCntrMax != 0 && p_spv->v.turnoffol_duration.faultCntr >= p_spv->p.turnoffol_duration.faultCntrMax)
            {
                p_spv->v.turnoffol_duration.faultCntr = p_spv->p.turnoffol_duration.faultCntrMax;
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
            }
            else
            {
                /* do nothing */
            }
        }
    }
}

static void fsOverLoadCheck(fsSpv_t *p_spv)
{
    mcc_t _iamp2;

    if(p_spv->p.over_load.enabled == MC_FALSE)
    {
        p_spv->v.over_load.faultCntr = 0u;
        
        return;
    }
    else
    {    
         _iamp2 = p_spv->v.iamp2;

        p_spv->v.over_load.intg += _iamp2 - p_spv->p.over_load.minCurrent;
        if(p_spv->v.over_load.intg < 0.0f)
            p_spv->v.over_load.intg = 0.0f;

        if(p_spv->v.over_load.intg > p_spv->p.over_load.intgMax)
        {
            p_spv->v.halt_duration = p_spv->p.over_load.duration;
            p_spv->v.over_load.flag = MC_TRUE;
                        
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_OVER_LOAD;
                        
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;                
                        
            p_spv->v.over_load.faultCntr++;
            if(p_spv->p.over_load.faultCntrMax != 0 && p_spv->v.over_load.faultCntr >= p_spv->p.over_load.faultCntrMax)
            {
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
                p_spv->v.over_load.faultCntr = p_spv->p.current_integral.faultCntrMax;
                       
                return;
            }
        }
    }
}

static void ThermalProtectionHandlerReset(fsSpv_t *p_spv)
{
    p_spv->v.thermalProtection.rref = p_spv->p.thermalProtection.rref_phase + p_spv->p.thermalProtection.rref_cable;
    
    p_spv->v.thermalProtection.pDeltaTempMax = p_spv->p.thermalProtection.pTempMax - p_spv->p.thermalProtection.tref;
    p_spv->v.thermalProtection.mDeltaTempMax = p_spv->p.thermalProtection.mTempMax - p_spv->p.thermalProtection.tref;
    p_spv->v.thermalProtection.tempFocScale = (p_spv->p.thermalProtection.mTempMax * 3.0f) / 2.0f;
    
    p_spv->v.thermalProtection.rcoeff = p_spv->p.thermalProtection.wcoeff * p_spv->v.thermalProtection.rref;
    p_spv->v.thermalProtection.pcoeff_phase = p_spv->v.thermalProtection.mDeltaTempMax / p_spv->p.thermalProtection.pRefPowerMax /
        p_spv->p.thermalProtection.currentLimitCompCoeff;
    p_spv->v.thermalProtection.A2washingcoeff = p_spv->p.thermalProtection.A2washingcoeff / 2.0 / p_spv->v.thermalProtection.tempFocScale;
    p_spv->v.thermalProtection.pHwPowerMax = p_spv->p.thermalProtection.maxSampledVoltage * p_spv->p.thermalProtection.maxSampledCurrent * 
        (sqrt(3)/3.0f) * (3.0f/2.0f);
    p_spv->v.thermalProtection.B1param = p_spv->p.thermalProtection.tempCompCoeff * p_spv->v.thermalProtection.tempFocScale /  2.0f /
        p_spv->v.thermalProtection.pHwPowerMax * 0.079897f;

    p_spv->v.thermalProtection.mt_delta = 0.0f;
    
    p_spv->v.thermalProtection.i2averaged = MC_FALSE;
    
    p_spv->v.thermalProtection.tdelta.u = 0.0f;
    p_spv->v.thermalProtection.tdelta.v = 0.0f;
    p_spv->v.thermalProtection.tdelta.w = 0.0f;
}

static void ThermalProtectionHandlerFL(fsSpv_t *p_spv)
{
    if((p_spv->p.thermalProtection.enabled == MC_FALSE) || 
       (p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_PASSIVE))
    {
        return;
    }
    else
    {
        if((p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_REAL_TIME_CALC) || 
           (p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_REAL_TIME_CALC_WAIT))
        {
            p_spv->v.thermalProtection.iuvw = p_spv->v.iuvw;
            
            p_spv->v.thermalProtection.i2uvw_sum.u += p_spv->v.thermalProtection.iuvw.u * p_spv->v.thermalProtection.iuvw.u;
            p_spv->v.thermalProtection.i2uvw_sum.v += p_spv->v.thermalProtection.iuvw.v * p_spv->v.thermalProtection.iuvw.v;
            p_spv->v.thermalProtection.i2uvw_sum.w += p_spv->v.thermalProtection.iuvw.w * p_spv->v.thermalProtection.iuvw.w;
            
            p_spv->v.thermalProtection.scnt++;
            if(p_spv->v.thermalProtection.scnt >= p_spv->p.thermalProtection.scnt_max)
            {
                p_spv->v.thermalProtection.i2uvw_ave.u = p_spv->v.thermalProtection.i2uvw_sum.u / (mcc_t)p_spv->v.thermalProtection.scnt;
                p_spv->v.thermalProtection.i2uvw_ave.v = p_spv->v.thermalProtection.i2uvw_sum.v / (mcc_t)p_spv->v.thermalProtection.scnt;
                p_spv->v.thermalProtection.i2uvw_ave.w = p_spv->v.thermalProtection.i2uvw_sum.w / (mcc_t)p_spv->v.thermalProtection.scnt;
                
                p_spv->v.thermalProtection.i2uvw_sum.u = (mcc_t)0;
                p_spv->v.thermalProtection.i2uvw_sum.v = (mcc_t)0;
                p_spv->v.thermalProtection.i2uvw_sum.w = (mcc_t)0;
                p_spv->v.thermalProtection.scnt = (mccnt_t)0;
                p_spv->v.thermalProtection.i2averaged = MC_TRUE;
                
                p_spv->v.mcvars.mc_state->thermal_state = FS_THERMAL_STATE_REAL_TIME_CALC;
            }
            else
            {
                /* do nothing */
            }
        }
        else if((p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_1ST_ACTIVE) || 
                (p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_2ND_ACTIVE))
        {
            p_spv->v.thermalProtection.i = p_spv->v.mcvars.idq->d;
            p_spv->v.thermalProtection.i_sum += p_spv->v.thermalProtection.i;
            
            p_spv->v.thermalProtection.v = p_spv->v.mcvars.vdq->d;
            p_spv->v.thermalProtection.v_sum += p_spv->v.thermalProtection.v;
            
            p_spv->v.thermalProtection.r = p_spv->v.thermalProtection.v / p_spv->v.thermalProtection.i;
            p_spv->v.thermalProtection.r_sum += p_spv->v.thermalProtection.r;
            
            p_spv->v.thermalProtection.scnt++;
        }
        else
        {
            /* do nothing */
        }
    }
}

static void ThermalProtectionHandlerSL(fsSpv_t *p_spv)
{
    if(p_spv->p.thermalProtection.enabled == MC_FALSE)
    {
        p_spv->v.thermalProtection.flag = MC_FALSE;
        
        return;
    }
    else
    {
        if((p_spv->v.thermalProtection.mt_delta > p_spv->v.thermalProtection.mDeltaTempMax) ||
           (p_spv->v.thermalProtection.tdelta.u > p_spv->v.thermalProtection.pDeltaTempMax) ||
               (p_spv->v.thermalProtection.tdelta.v > p_spv->v.thermalProtection.pDeltaTempMax) ||
                   (p_spv->v.thermalProtection.tdelta.w > p_spv->v.thermalProtection.pDeltaTempMax)
                       )
        {
            p_spv->v.halt_duration = p_spv->p.thermalProtection.duration;
            p_spv->v.thermalProtection.flag = MC_TRUE;
            
            p_spv->v.generatedFaultsBinary &= 0x00;
            p_spv->v.generatedFaultsBinary |= FS_BINARY_FLAG_THERMAL_PROTECTION;
            
            p_spv->v.mc_fs_state = FS_STATE_FAULT;
            p_spv->v.fsTotalGeneratedFault++;
            
            p_spv->v.thermalProtection.faultCntr++;
            if(p_spv->p.thermalProtection.faultCntrMax != 0 && p_spv->v.thermalProtection.faultCntr >= p_spv->p.thermalProtection.faultCntrMax)
            {
                p_spv->v.thermalProtection.faultCntr = p_spv->p.thermalProtection.faultCntrMax;
                p_spv->v.fsFatalFaultFlag = MC_TRUE;
            }
            else
            {
                ThermalProtectionHandlerReset(p_spv);
            }
            
            return;
        }
        else if(((p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_REAL_TIME_CALC) || 
                 (p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_REAL_TIME_CALC_WAIT)) && 
                (p_spv->v.thermalProtection.i2averaged == MC_FALSE))
        {
            return;
        }
        else
        {
            p_spv->v.thermalProtection.i2averaged = MC_FALSE;
        }
        
        if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_REAL_TIME_CALC)
        {
            /*************** Calculate Phase Each Resistance **********************/
            p_spv->v.thermalProtection.ruvw.u = p_spv->v.thermalProtection.rref + 
                (p_spv->v.thermalProtection.tdelta.u * p_spv->v.thermalProtection.rcoeff);
            p_spv->v.thermalProtection.ruvw.v = p_spv->v.thermalProtection.rref + 
                (p_spv->v.thermalProtection.tdelta.v * p_spv->v.thermalProtection.rcoeff);
            p_spv->v.thermalProtection.ruvw.w = p_spv->v.thermalProtection.rref + 
                (p_spv->v.thermalProtection.tdelta.w * p_spv->v.thermalProtection.rcoeff);
            
            if((p_spv->v.thermalProtection.ruvw.u <= (mcr_t)(-100)) ||
               (p_spv->v.thermalProtection.ruvw.v <= (mcr_t)(-100)) ||
                   (p_spv->v.thermalProtection.ruvw.w <= (mcr_t)(-100)))
            {
                return;
            }
            else
            {
                /* do nothing */
            }
            
            /***** Calculate resistance dependent consumed power for each phase ***/
            p_spv->v.thermalProtection.puvw.u = p_spv->v.thermalProtection.ruvw.u * 
                p_spv->v.thermalProtection.i2uvw_ave.u;
            p_spv->v.thermalProtection.puvw.v = p_spv->v.thermalProtection.ruvw.v * 
                p_spv->v.thermalProtection.i2uvw_ave.v;
            p_spv->v.thermalProtection.puvw.w = p_spv->v.thermalProtection.ruvw.w * 
                p_spv->v.thermalProtection.i2uvw_ave.w;
            p_spv->v.thermalProtection.ptotal = p_spv->v.thermalProtection.puvw.u + p_spv->v.thermalProtection.puvw.v + 
                p_spv->v.thermalProtection.puvw.w;
            
            /************ Calculate delta temperature for each phase **************/
            p_spv->v.thermalProtection.tdelta.u += 0.0625f * (p_spv->v.thermalProtection.mt_delta + 
                (p_spv->v.thermalProtection.puvw.u * p_spv->v.thermalProtection.pcoeff_phase) - p_spv->v.thermalProtection.tdelta.u);
            p_spv->v.thermalProtection.tdelta.v += 0.0625f * (p_spv->v.thermalProtection.mt_delta + 
                (p_spv->v.thermalProtection.puvw.v * p_spv->v.thermalProtection.pcoeff_phase) - p_spv->v.thermalProtection.tdelta.v);
            p_spv->v.thermalProtection.tdelta.w += 0.0625f * (p_spv->v.thermalProtection.mt_delta + 
                (p_spv->v.thermalProtection.puvw.w * p_spv->v.thermalProtection.pcoeff_phase) - p_spv->v.thermalProtection.tdelta.w);
            
            /*************** Calculate thermal model parameters *******************/
            p_spv->v.thermalProtection.plosses_coeff_motor = (mcpow_t)1 + (p_spv->p.thermalProtection.scoeff * mathAbsoluteFlt(*p_spv->v.mcvars.spest));
            p_spv->v.thermalProtection.peffect = p_spv->v.thermalProtection.ptotal * p_spv->v.thermalProtection.plosses_coeff_motor;
            p_spv->v.thermalProtection.A2param = p_spv->p.thermalProtection.A2washingoffset + 
                (p_spv->v.thermalProtection.A2washingcoeff * p_spv->v.thermalProtection.mt_delta);
            
            /**************** Calculate delta motor temperature *******************/
            /** Power consumed by phase resistance increase power. On the other hand
            current delta motor temperature resists to the temperature rise *******/
            p_spv->v.thermalProtection.mt_delta += (p_spv->v.thermalProtection.peffect * p_spv->v.thermalProtection.B1param) - 
                (p_spv->v.thermalProtection.mt_delta * p_spv->v.thermalProtection.A2param);
            if(p_spv->v.thermalProtection.mt_delta < p_spv->p.thermalProtection.mt_delta_min)
            {
                p_spv->v.thermalProtection.mt_delta = p_spv->p.thermalProtection.mt_delta_min;
            }
            else if(p_spv->v.thermalProtection.mt_delta > p_spv->p.thermalProtection.mt_delta_max)
            {
                p_spv->v.thermalProtection.mt_delta = p_spv->p.thermalProtection.mt_delta_max;
            }
            else
            {
                /* do nothing */
            }
            p_spv->v.thermalProtection.mt = p_spv->v.thermalProtection.mt_delta + p_spv->p.thermalProtection.tref;
        }
        else if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_1ST_WAIT)
        {
            p_spv->v.thermalProtection.v_sum = (mcv_t)0;
            p_spv->v.thermalProtection.i_sum = (mcc_t)0;
            p_spv->v.thermalProtection.r_sum = (mcr_t)0;
            p_spv->v.thermalProtection.scnt = (mccnt_t)0;
        }
        else if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_1ST_COMPLETED)
        {
            if((mcv_t)p_spv->v.thermalProtection.scnt > 0)
            {
                p_spv->v.thermalProtection.v1 = p_spv->v.thermalProtection.v_sum / (mcv_t)p_spv->v.thermalProtection.scnt;
                p_spv->v.thermalProtection.i1 = p_spv->v.thermalProtection.i_sum / (mcc_t)p_spv->v.thermalProtection.scnt;
                p_spv->v.thermalProtection.r1 = p_spv->v.thermalProtection.r_sum / (mcr_t)p_spv->v.thermalProtection.scnt;
            }
            p_spv->v.thermalProtection.scnt1 = p_spv->v.thermalProtection.scnt;
        }
        else if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_2ND_WAIT)
        {
            p_spv->v.thermalProtection.v_sum = (mcv_t)0;
            p_spv->v.thermalProtection.i_sum = (mcc_t)0;
            p_spv->v.thermalProtection.r_sum = (mcr_t)0;
            p_spv->v.thermalProtection.scnt = (mccnt_t)0;
        }
        else if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_2ND_COMPLETED)
        {
            if((mcv_t)p_spv->v.thermalProtection.scnt > 0)
            {
                p_spv->v.thermalProtection.v2 = p_spv->v.thermalProtection.v_sum / (mcv_t)p_spv->v.thermalProtection.scnt;
                p_spv->v.thermalProtection.i2 = p_spv->v.thermalProtection.i_sum / (mcc_t)p_spv->v.thermalProtection.scnt;
                p_spv->v.thermalProtection.r2 = p_spv->v.thermalProtection.r_sum / (mcr_t)p_spv->v.thermalProtection.scnt;
            }
            p_spv->v.thermalProtection.scnt2 = p_spv->v.thermalProtection.scnt;
            
            p_spv->v.mcvars.mc_state->thermal_state = FS_THERMAL_STATE_RCALC_COMPLETED;
        }
        else if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_RCALC_COMPLETED)
        {
            if(p_spv->v.thermalProtection.scnt1 > 0 && p_spv->v.thermalProtection.scnt2 > 0)
            {
                p_spv->v.thermalProtection.rfinal = (p_spv->v.thermalProtection.v1 - p_spv->v.thermalProtection.v2) / 
                    (p_spv->v.thermalProtection.i1 - p_spv->v.thermalProtection.i2);
                p_spv->v.thermalProtection.voffset1 = p_spv->v.thermalProtection.v1 - (p_spv->v.thermalProtection.rfinal * p_spv->v.thermalProtection.i1);
                p_spv->v.thermalProtection.voffset2 = p_spv->v.thermalProtection.v2 - (p_spv->v.thermalProtection.rfinal * p_spv->v.thermalProtection.i2);
                p_spv->v.thermalProtection.voffset = (p_spv->v.thermalProtection.voffset1 + p_spv->v.thermalProtection.voffset2) / 2.0f;
                
                p_spv->v.thermalProtection.mt_delta0 = (p_spv->v.thermalProtection.rfinal - p_spv->v.thermalProtection.rref) / 
                    p_spv->v.thermalProtection.rref / p_spv->p.thermalProtection.wcoeff;
                p_spv->v.thermalProtection.mt_delta = p_spv->v.thermalProtection.mt_delta0;
                p_spv->v.thermalProtection.mt0 = p_spv->v.thermalProtection.mt_delta0 + p_spv->p.thermalProtection.tref;
                
                p_spv->v.thermalProtection.tdelta.u = p_spv->v.thermalProtection.mt_delta;
                p_spv->v.thermalProtection.tdelta.v = p_spv->v.thermalProtection.mt_delta;
                p_spv->v.thermalProtection.tdelta.w = p_spv->v.thermalProtection.mt_delta;
            }
            p_spv->v.thermalProtection.scnt = (mccnt_t)0;
            p_spv->v.thermalProtection.i2uvw_sum.u = (mcc_t)0;
            p_spv->v.thermalProtection.i2uvw_sum.v = (mcc_t)0;
            p_spv->v.thermalProtection.i2uvw_sum.w = (mcc_t)0;
            
            p_spv->v.mcvars.mc_state->thermal_state = FS_THERMAL_STATE_REAL_TIME_CALC_WAIT;
        }
        else if(p_spv->v.mcvars.mc_state->thermal_state == FS_THERMAL_STATE_PASSIVE)
        {
            ThermalProtectionHandlerReset(p_spv);
        }
        else
        {
            /* do nothing */
        }
    }
}


