/**
* @file foc.c
* @author Serkan BUYUKABALI
* @date 07.09.2020
* @brief FOC - Field Oriented Control
* 
* Source code includes definitions of FOC functions to be used for motor control layer. All field oriented control
* process is applied in this source code. FOC layer is not aware of motor control states. State flow is decided in 
* the motor control source code and written to related motor run-time variables. FOC flows according to this state
* flow variable.
* 
* @see rd-repo.arcelik.com/PE_WG-MotorTech/MCP
*/


#include <math.h>
#include "foc.h"

/**
* @brief Field orented control fast loop handler
*
* Fast loop handler is executed in high frequency motor control interrupt. This interrupt must have the highest 
* priority .All transformations like clark/park transforms and estimations are executed in this handler.
*
* @param        p_mpv    address of related motor parameter-variable set
*
* @return       none
*/

static void mcEstimateSpeedAndPosition(mcCpv_t* p_mpv);
static mcvc_t DCurrentDecoupling(mcCpv_t* p_mpv);
static mcvc_t QCurrentDecoupling(mcCpv_t* p_mpv);
static void ManipulateVectorsForTransition(mcCpv_t* p_mpv);
static void Modulation(mcCpv_t *p_mpv);
static void Mtpa(mcCpv_t *p_mpv);

void mcFocFastLoop(mcCpv_t* p_mpv)
{
    if (p_mpv->v.state == INIT || p_mpv->v.pwm_enabled == MC_FALSE)
    {
        p_mpv->v.spact = p_mpv->v.spref = p_mpv->v.spest = (mcsp_t)0.0;
        p_mpv->v.spref_abs = p_mpv->v.spact_abs = p_mpv->v.sperr_abs = (mcsp_t)0;
        p_mpv->v.spref_radps = p_mpv->v.spest_radps = p_mpv->v.spact_radps = (mcsp_t)0;

        return;
    }
    else
    {
        ;  /* do nothing */
    }
    
    mcEstimateSpeedAndPosition(p_mpv);
        
    if ((p_mpv->v.statecom & FOC_PARK_TRANSFORM) != (mcstc_t)0)
    {
        ParkTransform((mcab_t*)&p_mpv->v.iab, &p_mpv->v.thact, (mcdq_t*)&p_mpv->v.idq);
    }
    else
    {
        ;  /* do nothing */
    }

    if((p_mpv->v.statecom & FOC_FIELD_WEAKENING) != (mcstc_t)0)
    {
        if(p_mpv->v.spref_abs > p_mpv->p.fwp.spmin)
        {
            FieldWeakening(&p_mpv->p.fwp, p_mpv->v.vamp, p_mpv->v.vbus_filtered, 
                                               &p_mpv->v.fwv);
        }
        else if(p_mpv->v.spref_abs < p_mpv->p.fwp.sp_reset)
        {
            FieldWeakeningReset(&p_mpv->v.fwv, p_mpv->v.vamp);
        }
        else
        {
            ;  /* do nothing */
        }
    }
    else
    {
        ;  /* do nothing */
    }
    
    if((p_mpv->v.statecom & FOC_MTPA) != (mcstc_t)0)
    {
            Mtpa(p_mpv);
    }
    else
    {
        ;  /* do nothing */
    }

    if ((p_mpv->v.statecom & FOC_CURRENT_CONTROLLER) != (mcstc_t)0)
    {
        if((p_mpv->v.cc_manipulate.state == 1) && (p_mpv->v.state == RAMP))
        {
            ManipulateVectorsForTransition(p_mpv);
        }
        else
        {
        	if(p_mpv->v.state == RAMP)
            {
        	if(p_mpv->v.fwv.idref != (mcc_t)0 || p_mpv->v.mtpa.idref != (mcc_t)0)
        	{
        		p_mpv->v.idqref.d = p_mpv->v.fwv.idref + p_mpv->v.mtpa.idref;
        	}
            p_mpv->v.idqref.q = p_mpv->v.iqref_spcontroller + p_mpv->v.torqComp.compIq;
            }

            p_mpv->v.idqref.d = contSaturate(p_mpv->v.idqref.d, -p_mpv->p.idqrefmax.d, p_mpv->p.idqrefmax.d);
            p_mpv->v.idqref.q = contSaturate(p_mpv->v.idqref.q, -p_mpv->p.idqrefmax.q, p_mpv->p.idqrefmax.q);
                
            p_mpv->v.iref_amp2 = p_mpv->v.idqref.d * p_mpv->v.idqref.d + p_mpv->v.idqref.q * p_mpv->v.idqref.q;
            p_mpv->v.iref_amp = (mcc_t)sqrtf(p_mpv->v.iref_amp2);
            p_mpv->v.th_iref_vec = atan2f(p_mpv->v.idqref.q, p_mpv->v.idqref.d) * MC_RAD2DEG + p_mpv->v.thact.th;
            
            CurrentPiController(p_mpv);
        }
    }
    else
    {
        ;  /* do nothing */
    }
    
    if ((p_mpv->v.statecom & FOC_INVERSE_PARK_TRANSFORM) != (mcstc_t)0)
    {
        p_mpv->v.vamp = sqrtf(p_mpv->v.vdq.d * p_mpv->v.vdq.d + p_mpv->v.vdq.q * p_mpv->v.vdq.q);

        InvParkTransform((mcdq_t*)&p_mpv->v.vdq, &p_mpv->v.thact, (mcab_t*)&p_mpv->v.vab);
    }
    else
    {
        ;  /* do nothing */
    }
    
    if ((p_mpv->v.statecom & FOC_DCBUS_RIPPLE_ELEMINATION) != (mcstc_t)0)
    {
        DCBusRippleElimination(&p_mpv->v.vab, &p_mpv->v.vabComp, p_mpv->v.vbus_filtered);
    }
    else
    {
        ;  /* do nothing */
    }
    
    if ((p_mpv->v.statecom & FOC_SPACE_VECTOR_MODULATION) != (mcstc_t)0)
    {
        Modulation(p_mpv);
    }
    else
    {
        ;  /* do nothing */
    }
    
    if ((p_mpv->v.statecom & FOC_DEAD_TIME_COMPENSATION) != (mcstc_t)0)
    {
        DeadtimeCompensation(p_mpv);
    }
    else
    {
        ;  /* do nothing */
    }
}

/**
* @brief Field orented control slow loop handler
*
* Slow loop handler is executed in low frequency motor control interrupt. This interrupt must have the second 
* highest priority; interrupt for the fast loop handler must have the highest priority. Speed controllers and 
* ramp controllers are executed in this handler. State machine can be handled in this function as well
*
* @param        p_mpv    address of related motor parameter-variable set
*
* @return       none
*/
void mcFocSlowLoop(mcCpv_t* p_mpv)
{
    mcObserversHandlerSL(p_mpv);
    
    if ((p_mpv->v.statecom & FOC_RAMP_CONTROLLER) != (mcstc_t)0)
    {
        RampController(&p_mpv->p.rampParams, p_mpv->v.sprefin, p_mpv->v.rampin,
            &p_mpv->v.ramp, &p_mpv->v.spref, p_mpv->p.freqSL);
    }
    else
    {
        ;  /* do nothing */
    }
    
    if ((p_mpv->v.statecom & FOC_SPEED_CONTROLLER) != (mcstc_t)0)
    {
        if(p_mpv->v.bypass.spController == MC_FALSE)
        {
            SpeedPiController(p_mpv);
        }
    }
    else
    {
        p_mpv->v.iqref_spcontroller = 0.0f;
    }
}

/**
* @brief Ramp Controller
* 
* Ramp controller function calculates next target reference speed according to requested speed and ramp. Requested
* ramp is applied between maksimum and minimum ramps that are defined in motor related control parameters. This
* function must be executed in slow loop handler.
* 
* @param[in]    l_rampParams structed variable includes ramp related parameters like maximum and minimum ramp values
* @param[in]    l_sprefin    pointer for requested speed variable
* @param[in]    l_rampin     pointer for requsted ramp value
* @param[out]   l_spref      pointer for calculated next cycle target speed
* 
* @return       boolean value shows success status of the function.
*/
void RampController(const mcRmpCntParams_t* l_rampParams, mcsp_t l_sprefin,
    rmps_t l_rampin, rmps_t* l_ramp, mcsp_t *l_spref, freq_t l_freq)
{
    mcsp_t l_spref_temp;
    rmps_t l_ramp_temp, l_ramp_max, l_ramp_min;
    
    if((*l_spref != l_sprefin) && (l_rampin != (rmps_t)0))
    {
        l_ramp_temp = mathAbsoluteFlt(l_rampin);
        
        if(l_sprefin > *l_spref)
        {
            l_ramp_max = l_rampParams->rampmaxAcc;
            l_ramp_min = l_rampParams->rampminAcc;
        }
        else if(l_sprefin < *l_spref)
        {
            l_ramp_max = l_rampParams->rampmaxDec;
            l_ramp_min = l_rampParams->rampminDec;
        }
        else
        {
            ;   /* do nothing */
        }

        if(l_ramp_temp > l_ramp_max)
        {
            l_ramp_temp = l_ramp_max;
        }
        else if(l_ramp_temp < l_ramp_min)
        {
            l_ramp_temp = l_ramp_min;
        }
        else
        {
            ;   /* do nothing */
        }

        if(l_ramp_temp < (rmps_t)0.0f)
            *l_ramp = (rmps_t)(-1) * l_ramp_temp;
        else
            *l_ramp = l_ramp_temp;
            
        l_ramp_temp = *l_ramp / (rmps_t)l_freq;
        l_spref_temp = *l_spref;
        
        if(l_sprefin > *l_spref)
        {
            l_spref_temp += l_ramp_temp;
            
            if(l_spref_temp > l_sprefin)
            {
                l_spref_temp = l_sprefin;
            }
            else
            {
                ;   /* do nothing */
            }
        }
        else if(l_sprefin < *l_spref)
        {
            l_spref_temp -= l_ramp_temp;
            
            if(l_spref_temp < l_sprefin)
            {
                l_spref_temp = l_sprefin;
            }
            else
            {
                ;   /* do nothing */
            }
        }
        else
        {
            ;   /* do nothing */
        }
        
        *l_spref = l_spref_temp;
    }    
    else
    {
        ;  /* do nothing */
    }
}

/**
* @brief Clark transform for field oriented control
* 
* Function transforms three phase current or voltage vectors to alpha-beta vectors. Results are divided to sqrt(3)
* to decrease execution time. Also resultant vector will give information about the pick values of phase 
* sinusoidals
*
* @param[in]    l_iuvw   pointer of the structed variable that includes phase vector amplitudes
* @param[out]   l_iab    pointer of the structed variable that includes alpha-beta vector amplitudes
*
* @return       none
*/
void ClarkTransform(const mcuvw_t* l_iuvw, mcab_t* l_iab)
{
    l_iab->a = l_iuvw->u;
    l_iab->b = (l_iuvw->v - l_iuvw->w) / (mcc_t)MC_SQRT_3;
}

/**
* @brief Inverse clark transform for field oriented control
* 
* Function transforms three phase current or voltage vectors to alpha-beta vectors. Results are divided to sqrt(3)
* to decrease execution time. Also resultant vector will give information about the pick values of phase 
* sinusoidals
*
* @param[in]   l_iab    pointer of the structed variable that includes alpha-beta vector amplitudes
* @param[out]  l_iuvw   pointer of the structed variable that will includes phase vector amplitudes
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void InvClarkTransform(const mcab_t* l_iab, mcuvw_t* l_iuvw)
{
    l_iuvw->u = l_iab->a;
    l_iuvw->v = ((-l_iab->a) + ((mcc_t)MC_SQRT_3 * l_iab->b)) / (mcc_t)2;
    l_iuvw->w = ((-l_iab->a) - ((mcc_t)MC_SQRT_3 * l_iab->b)) / (mcc_t)2;
}

/**
* @brief Park transform for field oriented control
*
* Function transforms alpha-beta current or voltage vectors to dq rotating vectors. Resultant vectors of alpha-beta 
* and dq vectors should be the same. d and q vector amplitudes will change according to vector position
*
* @param[in]    l_ab     pointer of the structed variable that includes alpha-beta vector amplitudes
* @param[in]    l_thact  pointer of the variable that is the position (angle) of the dq rotating system
* @param[out]   l_dq     pointer of the structed variable that includes dq values
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void ParkTransform(const mcab_t* l_ab, const th_t* l_thact, mcdq_t* l_dq)
{
    l_dq->d = (l_ab->a * l_thact->cos) + (l_ab->b * l_thact->sin);
    l_dq->q = ((-l_ab->a) * l_thact->sin) + (l_ab->b * l_thact->cos);
}

/**
* @brief Inverse park transform for field oriented control
*
* Function transforms dq rotating vector values to alpha-beta vectors. Resultant vectors of dq
* and alpha-beta vectors should be the same.
*
* @param[in]    l_dq     pointer of the structed variable that includes dq values
* @param[in]    l_thact  pointer of the variable that is the position (angle) of the dq rotating system
* @param[out]   l_ab     pointer of the structed variable that includes alpha-beta vector amplitudes
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void InvParkTransform(const mcdq_t* l_dq, const th_t* l_thact, mcab_t* l_ab)
{
    l_ab->a = (l_dq->d * l_thact->cos) - (l_dq->q * l_thact->sin);
    l_ab->b = (l_dq->d * l_thact->sin) + (l_dq->q * l_thact->cos);
}

/**
* @brief Current MC_PI controller for field oriented control
*
* Calculates the dq voltage values to have desired dq current values :
* Calculates unsaturated dq voltage values using pi controller. Control object is dq currents
* Decouples unsaturated dq voltages according to motor model
* Apply control for saturation limits
* Apply antiwindup to prevent unstable states
*
* @param        p_mpv    address of related motor parameter-variable set
*
* @return       none
*/
void CurrentPiController(mcCpv_t* p_mpv)
{
    mcc_t error_id, error_iq;

    error_id = p_mpv->v.idqref.d - p_mpv->v.idq.d;
    error_iq = p_mpv->v.idqref.q - p_mpv->v.idq.q;

    p_mpv->v.error_filtered_id += p_mpv->p.zcCoeffDCurrent * (error_id - p_mpv->v.error_filtered_id);
    p_mpv->v.error_filtered_iq += p_mpv->p.zcCoeffQCurrent * (error_iq - p_mpv->v.error_filtered_iq);
    
    p_mpv->v.vdq_phase.d = contDiscretePiController(p_mpv->v.error_filtered_id, &p_mpv->v.piDCurrent);
    p_mpv->v.vdq_phase.q = contDiscretePiController(p_mpv->v.error_filtered_iq, &p_mpv->v.piQCurrent);

    /* coupling process */
    p_mpv->v.vdq_decoupler.d = DCurrentDecoupling(p_mpv);
    p_mpv->v.vdq_decoupler.q = QCurrentDecoupling(p_mpv) + (p_mpv->v.spref_radps * p_mpv->p.phys.Ke);

    p_mpv->v.vdq_unsaturated.d = p_mpv->v.vdq_phase.d - p_mpv->v.vdq_decoupler.d;
    p_mpv->v.vdq_unsaturated.q = p_mpv->v.vdq_phase.q + p_mpv->v.vdq_decoupler.q;

    p_mpv->v.vdq.d = contSaturate(p_mpv->v.vdq_unsaturated.d, -p_mpv->p.vdqmax.d, p_mpv->p.vdqmax.d);
    p_mpv->v.vdq.q = contSaturate(p_mpv->v.vdq_unsaturated.q, -p_mpv->p.vdqmax.q, p_mpv->p.vdqmax.q);
}

static mcvc_t DCurrentDecoupling(mcCpv_t* p_mpv)
{
    if(p_mpv->p.decouplingMethod == MC_DECOUPLING_METHOD_CCALC)
    {
        p_mpv->v.vdq_decoupler_dq.d += p_mpv->v.decoupling_lpfcoeff.d * (p_mpv->v.inductanceCorrectionCoeff.d * p_mpv->v.idqref.q * p_mpv->v.spref_radps * p_mpv->p.phys.Lq - p_mpv->v.vdq_decoupler_dq.d);
    }
    else if(p_mpv->p.decouplingMethod == MC_DECOUPLING_METHOD_VCALC)
    {
        p_mpv->v.vdq_decoupler_dq.d = p_mpv->v.vdq_decoupler_dq.d * (mcv_t)0.9906 + p_mpv->v.vdq_phase.q * p_mpv->v.spref_radps * p_mpv->p.phys.Lq * (mcv_t)0.00292;
    }
    else /* if(p_mpv->p.decouplingMethod == MC_DECOUPLING_METHOD_NCALC) */
    {
        p_mpv->v.vdq_decoupler_dq.d = 0.0f;
    }

    return p_mpv->v.vdq_decoupler_dq.d;
}

static mcvc_t QCurrentDecoupling(mcCpv_t* p_mpv)
{
    if(p_mpv->p.decouplingMethod == MC_DECOUPLING_METHOD_CCALC)
    {
        p_mpv->v.vdq_decoupler_dq.q +=  p_mpv->v.decoupling_lpfcoeff.q * (p_mpv->v.inductanceCorrectionCoeff.q * p_mpv->v.idqref.d * p_mpv->v.spref_radps * p_mpv->p.phys.Ld - p_mpv->v.vdq_decoupler_dq.q);
    }
    else if(p_mpv->p.decouplingMethod == MC_DECOUPLING_METHOD_VCALC)
    {
        p_mpv->v.vdq_decoupler_dq.q = p_mpv->v.vdq_decoupler_dq.q * (mcv_t)0.9922 + p_mpv->v.vdq_phase.d * p_mpv->v.spref_radps * p_mpv->p.phys.Ld * (mcv_t)0.00249;
    }
    else /* if(p_mpv->p.decouplingMethod == MC_DECOUPLING_METHOD_NCALC) */
    {
        p_mpv->v.vdq_decoupler_dq.q = (mcv_t)0;
    }
    
    return p_mpv->v.vdq_decoupler_dq.q;
}

static void ManipulateVectorsForTransition(mcCpv_t* p_mpv)
{
    p_mpv->v.spref = p_mpv->v.spest;
    p_mpv->v.spref_abs = mathAbsoluteFlt(p_mpv->v.spref);
    p_mpv->v.idqref.d = p_mpv->v.idq.d;
    p_mpv->v.idqref.q = p_mpv->v.idq.q;
    p_mpv->v.piSpeed.integ = p_mpv->v.idqref.q;
    p_mpv->v.iqref_spcontroller = p_mpv->v.piSpeed.integ;
	    
    ParkTransform(&p_mpv->v.vab, &p_mpv->v.thest, &p_mpv->v.cc_manipulate.vdq);

    p_mpv->v.vdq_decoupler.d = DCurrentDecoupling(p_mpv);
    p_mpv->v.vdq_decoupler.q = QCurrentDecoupling(p_mpv) + (p_mpv->v.spref_radps * p_mpv->p.phys.Ke);

    p_mpv->v.vdq_phase.d = p_mpv->v.cc_manipulate.vdq.d + p_mpv->v.vdq_decoupler.d;
    p_mpv->v.vdq_phase.q = p_mpv->v.cc_manipulate.vdq.q - p_mpv->v.vdq_decoupler.q;
    
    p_mpv->v.piDCurrent.integ = p_mpv->v.vdq_phase.d;
    p_mpv->v.piQCurrent.integ = p_mpv->v.vdq_phase.q;

    p_mpv->v.vdq.d = p_mpv->v.cc_manipulate.vdq.d;
    p_mpv->v.vdq.q = p_mpv->v.cc_manipulate.vdq.q;
    
    p_mpv->v.cc_manipulate.state = 0;
}

/**
* @brief DC BUS ripple elemination
*
* Apply correction to alpha-beta voltage vectors according to DC BUS voltage value
*
* @param[in]    l_vab        pointer of the structed voltage variable that includes alpha-beta voltag evector amplitudes
* @param[out]   l_vabComp    pointer of the variable that has the compansated alpha-beta voltages
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
mc_bool_t DCBusRippleElimination(const mcab_t* l_vab, mcab_t* l_vabComp, mcv_t l_dcBaraVoltage)
{
    if(l_dcBaraVoltage <= (mcv_t)0)
    {
        l_vabComp->a = (mcv_t)0;
        l_vabComp->b = (mcv_t)0;
        
        return MC_FALSE;
    }
    else
    {
        if(l_vab->a > l_dcBaraVoltage)
        {
            l_vabComp->a = (mcv_t)1;
        }
        else if(l_vab->a < -l_dcBaraVoltage)
        {
            l_vabComp->a = (mcv_t)-1;
        }
        else
        {
            /* l_vabComp->a = l_vab->a / (l_dcBaraVoltage * (2 / 3) * MC_SQRT_3 / 2) */
            l_vabComp->a = l_vab->a / (l_dcBaraVoltage / MC_SQRT_3);
        }
        
        if(l_vab->b > l_dcBaraVoltage)
        {
            l_vabComp->b = (mcv_t)1;
        }
        else if(l_vab->b < -l_dcBaraVoltage)
        {
            l_vabComp->b = (mcv_t)-1;
        }
        else
        {
            /* l_vabComp->b = l_vab->b / (l_dcBaraVoltage * (2 / 3) * MC_SQRT_3 / 2) */
            l_vabComp->b = l_vab->b / (l_dcBaraVoltage / (mcv_t)MC_SQRT_3);
        }
        
        return MC_TRUE;
    }
}

/**
* @brief Space Vector Modulation
*
* Applience of space vector modulation technique to calculate pwm duties 
*
* @param[in]    l_vabComp    pointer of the variable that has the compansated alpha-beta voltages
* @param[out]   l_sector     denotes the current sector for the desired vector
* @param[out]   l_pwm_duty   address of pointer array that has the aaddresses of pwm duty variables
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void SpaceVectorModulation(const mcab_t* l_vabComp, sector_t *l_sector, duvw_t* l_pwm_duty)
{
    mcv_t uref1, uref2, uref3;
    mcv_t X, Y, Z;
    timing_t t_1, t_2;
    timing_t t1, t2, t3;
    duty_t pwma, pwmb, pwmc;
    sector_t sector_;
    
    const timing_t T = (timing_t)1;
    
    uref1 = l_vabComp->b;
    uref2 = ((-l_vabComp->b) + (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    uref3 = ((-l_vabComp->b) - (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    
    X = l_vabComp->b;
    Y = (l_vabComp->b + (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    Z = (l_vabComp->b - (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    
    if(uref3 <= (mcv_t)0)
    {
        if(uref2 > (mcv_t)0)
        {
            if(uref1 < (mcv_t)0)
            {
                sector_ = (sector_t)6;
                t_1 = (timing_t)(-X);
                t_2 = (timing_t)Y;
            }
            else
            {
                sector_ = (sector_t)1;
                t_1 = (timing_t)X;
                t_2 = (timing_t)(-Z);
            }
        }
        else
        {
            sector_ = (sector_t)2;
            t_1 = (timing_t)Y;
            t_2 = (timing_t)Z;
        }
    }
    else
    {
        if(uref2 <= (mcv_t)0)
        {
            if(uref1 > (mcv_t)0)
            {
                sector_ = (sector_t)3;
                t_1 = (timing_t)(-Y);
                t_2 = (timing_t)X;
            }
            else
            {
                sector_ = (sector_t)4;
                t_1 = (timing_t)Z;
                t_2 = (timing_t)(-X);
            }
        }
        else
        {
            sector_ = (sector_t)5;
            t_1 = (timing_t)(-Z);
            t_2 = (timing_t)(-Y);
        }
    }
    
    t1 = (T - t_1 - t_2) / (timing_t)2;
    t2 = t1 + t_1;
    t3 = t2 + t_2;
    
/*    t1_duty = (duty_t)(t1 * DUTY_FULL_SCALE) */
/*    t2_duty = (duty_t)(t2 * DUTY_FULL_SCALE) */
/*    t3_duty = (duty_t)(t3 * DUTY_FULL_SCALE) */
    
    if(sector_ == (sector_t)1)
    {
        pwma = (duty_t)t3;
        pwmb = (duty_t)t2;
        pwmc = (duty_t)t1;
    }
    else if(sector_ == (sector_t)2)
    {
        pwma = (duty_t)t2;
        pwmb = (duty_t)t3;
        pwmc = (duty_t)t1;
    }
    else if(sector_ == (sector_t)3)
    {
        pwma = (duty_t)t1;
        pwmb = (duty_t)t3;
        pwmc = (duty_t)t2;
    }
    else if(sector_ == (sector_t)4)
    {
        pwma = (duty_t)t1;
        pwmb = (duty_t)t2;
        pwmc = (duty_t)t3;
    }
    else if(sector_ == (sector_t)5)
    {
        pwma = (duty_t)t2;
        pwmb = (duty_t)t1;
        pwmc = (duty_t)t3;
    }
    else if(sector_ == (sector_t)6)
    {
        pwma = (duty_t)t3;
        pwmb = (duty_t)t1;
        pwmc = (duty_t)t2;
    }
    else
    {
        ;  /* do nothing */
    }
    
    *l_sector = sector_;
    
    if(pwma < (duty_t)0)
    {
         l_pwm_duty->u = (duty_t)0;
    }
    else if(pwma > (duty_t)1)
    {
        l_pwm_duty->u = (duty_t)1;
    }
    else
    {
        l_pwm_duty->u = pwma;
    }
    
    if(pwmb < (duty_t)0)
    {
        l_pwm_duty->v = (duty_t)0;
    }
    else if(pwmb > (duty_t)1)
    {
        l_pwm_duty->v = (duty_t)1;
    }
    else
    {
        l_pwm_duty->v = pwmb;
    }
    
    if(pwmc < (duty_t)0)
    {
        l_pwm_duty->w = (duty_t)0;
    }
    else if(pwmc > (duty_t)1)
    {
        l_pwm_duty->w = (duty_t)1;
    }
    else
    {
        l_pwm_duty->w = pwmc;
    }
}

/**
* @brief Space Vector Modulation for 0th vector
*       High side transistors are fully closed for 2 sectors
*
* Applience of space vector modulation technique to calculate pwm duties
*
* @param[in]    l_vabComp    pointer of the variable that has the compansated alpha-beta voltages
* @param[out]   l_sector     denotes the current sector for the desired vector
* @param[out]   l_pwm_duty   address of pointer array that has the aaddresses of pwm duty variables
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void SpaceVectorModulationU0(const mcab_t* l_vabComp, sector_t *l_sector, duvw_t* l_pwm_duty)
{
    mcv_t uref1, uref2, uref3;
    mcv_t X, Y, Z;
    timing_t t_1, t_2;
    timing_t t1, t2, t3;
    duty_t pwma, pwmb, pwmc;
    sector_t sector_;

    uref1 = l_vabComp->b;
    uref2 = ((-l_vabComp->b) + (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    uref3 = ((-l_vabComp->b) - (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;

    X = l_vabComp->b;
    Y = (l_vabComp->b + (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    Z = (l_vabComp->b - (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;

    if(uref3 <= (mcv_t)0)
    {
        if(uref2 > (mcv_t)0)
        {
            if(uref1 < (mcv_t)0)
            {
                sector_ = (sector_t)6;
                t_1 = (timing_t)(-X);
                t_2 = (timing_t)Y;
            }
            else
            {
                sector_ = (sector_t)1;
                t_1 = (timing_t)X;
                t_2 = (timing_t)(-Z);
            }
        }
        else
        {
            sector_ = (sector_t)2;
            t_1 = (timing_t)Y;
            t_2 = (timing_t)Z;
        }
    }
    else
    {
        if(uref2 <= (mcv_t)0)
        {
            if(uref1 > (mcv_t)0)
            {
                sector_ = (sector_t)3;
                t_1 = (timing_t)(-Y);
                t_2 = (timing_t)X;
            }
            else
            {
                sector_ = (sector_t)4;
                t_1 = (timing_t)Z;
                t_2 = (timing_t)(-X);
            }
        }
        else
        {
            sector_ = (sector_t)5;
            t_1 = (timing_t)(-Z);
            t_2 = (timing_t)(-Y);
        }
    }

    t1 = 0;
    t2 = t1 + t_1;
    t3 = t2 + t_2;

/*    t1_duty = (duty_t)(t1 * DUTY_FULL_SCALE) */
/*    t2_duty = (duty_t)(t2 * DUTY_FULL_SCALE) */
/*    t3_duty = (duty_t)(t3 * DUTY_FULL_SCALE) */

    if(sector_ == (sector_t)1)
    {
        pwma = (duty_t)t3;
        pwmb = (duty_t)t2;
        pwmc = (duty_t)t1;
    }
    else if(sector_ == (sector_t)2)
    {
        pwma = (duty_t)t2;
        pwmb = (duty_t)t3;
        pwmc = (duty_t)t1;
    }
    else if(sector_ == (sector_t)3)
    {
        pwma = (duty_t)t1;
        pwmb = (duty_t)t3;
        pwmc = (duty_t)t2;
    }
    else if(sector_ == (sector_t)4)
    {
        pwma = (duty_t)t1;
        pwmb = (duty_t)t2;
        pwmc = (duty_t)t3;
    }
    else if(sector_ == (sector_t)5)
    {
        pwma = (duty_t)t2;
        pwmb = (duty_t)t1;
        pwmc = (duty_t)t3;
    }
    else if(sector_ == (sector_t)6)
    {
        pwma = (duty_t)t3;
        pwmb = (duty_t)t1;
        pwmc = (duty_t)t2;
    }
    else
    {
        ;  /* do nothing */
    }

    *l_sector = sector_;

    if(pwma < (duty_t)0)
    {
         l_pwm_duty->u = (duty_t)0;
    }
    else if(pwma > (duty_t)1)
    {
        l_pwm_duty->u = (duty_t)1;
    }
    else
    {
        l_pwm_duty->u = pwma;
    }

    if(pwmb < (duty_t)0)
    {
        l_pwm_duty->v = (duty_t)0;
    }
    else if(pwmb > (duty_t)1)
    {
        l_pwm_duty->v = (duty_t)1;
    }
    else
    {
        l_pwm_duty->v = pwmb;
    }

    if(pwmc < (duty_t)0)
    {
        l_pwm_duty->w = (duty_t)0;
    }
    else if(pwmc > (duty_t)1)
    {
        l_pwm_duty->w = (duty_t)1;
    }
    else
    {
        l_pwm_duty->w = pwmc;
    }
}

/**
* @brief Space Vector Modulation for 7th vector
*       Low side transistors are fully closed for 2 sectors
*
* Applience of space vector modulation technique to calculate pwm duties
*
* @param[in]    l_vabComp    pointer of the variable that has the compansated alpha-beta voltages
* @param[out]   l_sector     denotes the current sector for the desired vector
* @param[out]   l_pwm_duty   address of pointer array that has the aaddresses of pwm duty variables
*
* @return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void SpaceVectorModulationU7(const mcab_t* l_vabComp, sector_t *l_sector, duvw_t* l_pwm_duty)
{
    mcv_t uref1, uref2, uref3;
    mcv_t X, Y, Z;
    timing_t t_1, t_2;
    timing_t t1, t2, t3;
    duty_t pwma, pwmb, pwmc;
    sector_t sector_;

    const timing_t T = (timing_t)1;

    uref1 = l_vabComp->b;
    uref2 = ((-l_vabComp->b) + (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    uref3 = ((-l_vabComp->b) - (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;

    X = l_vabComp->b;
    Y = (l_vabComp->b + (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;
    Z = (l_vabComp->b - (MC_SQRT_3 * l_vabComp->a)) / (mcv_t)2;

    if(uref3 <= (mcv_t)0)
    {
        if(uref2 > (mcv_t)0)
        {
            if(uref1 < (mcv_t)0)
            {
                sector_ = (sector_t)6;
                t_1 = (timing_t)(-X);
                t_2 = (timing_t)Y;
            }
            else
            {
                sector_ = (sector_t)1;
                t_1 = (timing_t)X;
                t_2 = (timing_t)(-Z);
            }
        }
        else
        {
            sector_ = (sector_t)2;
            t_1 = (timing_t)Y;
            t_2 = (timing_t)Z;
        }
    }
    else
    {
        if(uref2 <= (mcv_t)0)
        {
            if(uref1 > (mcv_t)0)
            {
                sector_ = (sector_t)3;
                t_1 = (timing_t)(-Y);
                t_2 = (timing_t)X;
            }
            else
            {
                sector_ = (sector_t)4;
                t_1 = (timing_t)Z;
                t_2 = (timing_t)(-X);
            }
        }
        else
        {
            sector_ = (sector_t)5;
            t_1 = (timing_t)(-Z);
            t_2 = (timing_t)(-Y);
        }
    }

    t1 = T - t_1 - t_2;
    t2 = t1 + t_1;
    t3 = t2 + t_2;

/*    t1_duty = (duty_t)(t1 * DUTY_FULL_SCALE) */
/*    t2_duty = (duty_t)(t2 * DUTY_FULL_SCALE) */
/*    t3_duty = (duty_t)(t3 * DUTY_FULL_SCALE) */

    if(sector_ == (sector_t)1)
    {
        pwma = (duty_t)t3;
        pwmb = (duty_t)t2;
        pwmc = (duty_t)t1;
    }
    else if(sector_ == (sector_t)2)
    {
        pwma = (duty_t)t2;
        pwmb = (duty_t)t3;
        pwmc = (duty_t)t1;
    }
    else if(sector_ == (sector_t)3)
    {
        pwma = (duty_t)t1;
        pwmb = (duty_t)t3;
        pwmc = (duty_t)t2;
    }
    else if(sector_ == (sector_t)4)
    {
        pwma = (duty_t)t1;
        pwmb = (duty_t)t2;
        pwmc = (duty_t)t3;
    }
    else if(sector_ == (sector_t)5)
    {
        pwma = (duty_t)t2;
        pwmb = (duty_t)t1;
        pwmc = (duty_t)t3;
    }
    else if(sector_ == (sector_t)6)
    {
        pwma = (duty_t)t3;
        pwmb = (duty_t)t1;
        pwmc = (duty_t)t2;
    }
    else
    {
        ;  /* do nothing */
    }

    *l_sector = sector_;

    if(pwma < (duty_t)0)
    {
         l_pwm_duty->u = (duty_t)0;
    }
    else if(pwma > (duty_t)1)
    {
        l_pwm_duty->u = (duty_t)1;
    }
    else
    {
        l_pwm_duty->u = pwma;
    }

    if(pwmb < (duty_t)0)
    {
        l_pwm_duty->v = (duty_t)0;
    }
    else if(pwmb > (duty_t)1)
    {
        l_pwm_duty->v = (duty_t)1;
    }
    else
    {
        l_pwm_duty->v = pwmb;
    }

    if(pwmc < (duty_t)0)
    {
        l_pwm_duty->w = (duty_t)0;
    }
    else if(pwmc > (duty_t)1)
    {
        l_pwm_duty->w = (duty_t)1;
    }
    else
    {
        l_pwm_duty->w = pwmc;
    }
}
/**
* @brief Dead time compensation
*
* Function is used to compensate dead time for PWM duty cycles
* Common function that selects proper dead-time compensation method and
* than calls related function
*/
void DeadtimeCompensation(mcCpv_t* p_mpv)
{
    if(p_mpv->p.modulation.dtCompType == DTCOMP_BIDIRECTIONAL)
    {
        DeadtimeCompensation_BiDirectional(p_mpv);
    }
    else if(p_mpv->p.modulation.dtCompType == DTCOMP_UNIDIRECTIONAL_NEG)
    {
        DeadtimeCompensation_UniDirectionalNeg(p_mpv);
    }
    else if(p_mpv->p.modulation.dtCompType == DTCOMP_UNIDIRECTIONAL_POS)
    {
        DeadtimeCompensation_UniDirectionalPos(p_mpv);
    }
    else //if(p_mpv->p.modulation.dtCompType == DTCOMP_NONE)
    {
        ;   /* do nothing */
    }
}
/**
* @brief Dead time compensation
*
* Function is used to compensate dead time for PWM duty cycles
* Compensation is applied for both current values; positive and negative current values;
* means applied for both top and bottom transistors
*/
void DeadtimeCompensation_BiDirectional(mcCpv_t* p_mpv)
{
    duvw_t l_dutyOut;

    if(p_mpv->v.iuvw.u > (mcc_t)0.0f)
    {
        p_mpv->v.siuvw.u = (sign_t)1;
    }
    else if(p_mpv->v.iuvw.u < (mcc_t)0.0f)
    {
        p_mpv->v.siuvw.u = (sign_t)(-1);
    }
    else
    {
        p_mpv->v.siuvw.u = (sign_t)0;
    }
    
    if(p_mpv->v.iuvw.v > (mcc_t)0.0f)
    {
        p_mpv->v.siuvw.v = (sign_t)1;
    }
    else if(p_mpv->v.iuvw.v < (mcc_t)0.0f)
    {
        p_mpv->v.siuvw.v = (sign_t)(-1);
    }
    else
    {
        p_mpv->v.siuvw.v = (sign_t)0;
    }
    
    if(p_mpv->v.iuvw.w > (mcc_t)0.0f)
    {
        p_mpv->v.siuvw.w = (sign_t)1;
    }
    else if(p_mpv->v.iuvw.w < (mcc_t)0.0f)
    {
        p_mpv->v.siuvw.w = (sign_t)(-1);
    }
    else
    {
        p_mpv->v.siuvw.w = (sign_t)0;
    }

    l_dutyOut.u = p_mpv->v.duvw.u + ((duty_t)p_mpv->v.siuvw.u * p_mpv->p.modulation.deadtime);
    l_dutyOut.v = p_mpv->v.duvw.v + ((duty_t)p_mpv->v.siuvw.v * p_mpv->p.modulation.deadtime);
    l_dutyOut.w = p_mpv->v.duvw.w + ((duty_t)p_mpv->v.siuvw.w * p_mpv->p.modulation.deadtime);

    if(l_dutyOut.u > 1.0f)
        l_dutyOut.u = 1.0f;
    else if(l_dutyOut.u < 0.0f)
        l_dutyOut.u = 0.0f;

    if(l_dutyOut.v > 1.0f)
        l_dutyOut.v = 1.0f;
    else if(l_dutyOut.v < 0.0f)
        l_dutyOut.v = 0.0f;

    if(l_dutyOut.w > 1.0f)
        l_dutyOut.w = 1.0f;
    else if(l_dutyOut.w < 0.0f)
        l_dutyOut.w = 0.0f;

    p_mpv->v.duvw_comp.u = l_dutyOut.u;
    p_mpv->v.duvw_comp.v = l_dutyOut.v;
    p_mpv->v.duvw_comp.w = l_dutyOut.w;
}
/**
* @brief Dead time compensation
*
* Function is used to compensate dead time for PWM duty cycles
* Compensation is applied only for neagative current values; means applied only for
* when bottom transistors are active
*/
void DeadtimeCompensation_UniDirectionalNeg(mcCpv_t* p_mpv)
{
    duty_t l_deadtime_u, l_deadtime_v, l_deadtime_w;
    duvw_t l_dutyOut;

    l_deadtime_u = l_deadtime_v = l_deadtime_w = p_mpv->p.modulation.deadtime;

    if(p_mpv->v.iuvw.u < (mcc_t)(-p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.u = (sign_t)(-1);
    }
    else if(p_mpv->v.iuvw.u > (mcc_t)(p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.u = (sign_t)0;
    }
    else if(p_mpv->v.iuvw_prev.u < (mcc_t)(-p_mpv->p.modulation.dt_trans_duty) && p_mpv->v.iuvw.u >= (mcc_t)(-p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.u = (sign_t)0;
    }
    else if(p_mpv->v.iuvw_prev.u > (mcc_t)p_mpv->p.modulation.dt_trans_duty && p_mpv->v.iuvw.u <= (mcc_t)p_mpv->p.modulation.dt_trans_duty)
    {
        p_mpv->v.siuvw.u = (sign_t)(-1);
    }
    p_mpv->v.iuvw_prev.u = p_mpv->v.iuvw.u;

    if(p_mpv->v.iuvw.v < (mcc_t)(-p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.v = (sign_t)(-1);
    }
    else if(p_mpv->v.iuvw.v > (mcc_t)p_mpv->p.modulation.dt_trans_duty)
    {
        p_mpv->v.siuvw.v = (sign_t)0;
    }
    else if(p_mpv->v.iuvw_prev.v < (mcc_t)(-p_mpv->p.modulation.dt_trans_duty) && p_mpv->v.iuvw.v >= (mcc_t)(-p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.v = (sign_t)0;
    }
    else if(p_mpv->v.iuvw_prev.v > (mcc_t)p_mpv->p.modulation.dt_trans_duty && p_mpv->v.iuvw.v <= (mcc_t)p_mpv->p.modulation.dt_trans_duty)
    {
        p_mpv->v.siuvw.v = (sign_t)(-1);
    }
    p_mpv->v.iuvw_prev.v = p_mpv->v.iuvw.v;

    if(p_mpv->v.iuvw.w < (mcc_t)(-p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.w = (sign_t)(-1);
    }
    else if(p_mpv->v.iuvw.w > (mcc_t)p_mpv->p.modulation.dt_trans_duty)
    {
        p_mpv->v.siuvw.w = (sign_t)0;
    }
    else if(p_mpv->v.iuvw_prev.w < (mcc_t)(-p_mpv->p.modulation.dt_trans_duty) && p_mpv->v.iuvw.w >= (mcc_t)(-p_mpv->p.modulation.dt_trans_duty))
    {
        p_mpv->v.siuvw.w = (sign_t)0;
    }
    else if(p_mpv->v.iuvw_prev.w > (mcc_t)p_mpv->p.modulation.dt_trans_duty && p_mpv->v.iuvw.w <= (mcc_t)p_mpv->p.modulation.dt_trans_duty)
    {
        p_mpv->v.siuvw.w = (sign_t)(-1);
    }
    p_mpv->v.iuvw_prev.w = p_mpv->v.iuvw.w;
    
    l_dutyOut.u = p_mpv->v.duvw.u + ((duty_t)p_mpv->v.siuvw.u * l_deadtime_u);
    l_dutyOut.v = p_mpv->v.duvw.v + ((duty_t)p_mpv->v.siuvw.v * l_deadtime_v);
    l_dutyOut.w = p_mpv->v.duvw.w + ((duty_t)p_mpv->v.siuvw.w * l_deadtime_w);

    if(l_dutyOut.u < (duty_t)0)
        l_dutyOut.u = (duty_t)0;
    else if(l_dutyOut.u > (duty_t)1)
        l_dutyOut.u = (duty_t)1;

    if(l_dutyOut.v < (duty_t)0)
        l_dutyOut.v = (duty_t)0;
    else if(l_dutyOut.v > (duty_t)1)
        l_dutyOut.v = (duty_t)1;

    if(l_dutyOut.w < (duty_t)0)
        l_dutyOut.w = (duty_t)0;
    else if(l_dutyOut.w > (duty_t)1)
        l_dutyOut.w = (duty_t)1;

    p_mpv->v.duvw_comp.u = l_dutyOut.u;
    p_mpv->v.duvw_comp.v = l_dutyOut.v;
    p_mpv->v.duvw_comp.w = l_dutyOut.w;
}
/**
* @brief Dead time compensation
*
* Function is used to compensate dead time for PWM duty cycles
* Compensation is applied only for neagative current values; means applied only for
* when top transistors are active
*/
void DeadtimeCompensation_UniDirectionalPos(mcCpv_t* p_mpv)
{

}
/**
* @brief Speed MC_PI controller
*
* Calculates target iqref for the decided reference speed. Gap between decided reference speed and current speed is the
* input of the controller. Ambition to make difference zero.
*
* @param[in]    p_mpv       address of the motor control object
*
@return       boolean value shows success status of the function; any case returns MC_TRUE
*/
void SpeedPiController(mcCpv_t* p_mpv)
{
    mcsp_t error;
    mcc_t _iqref_temp;
    mcc_t l_iqControllerIntegPrev;
    
    l_iqControllerIntegPrev = p_mpv->v.piSpeed.integ;
    error = p_mpv->v.spref_radps - p_mpv->v.spest_radps;
    _iqref_temp = contDiscretePiController(error, &p_mpv->v.piSpeed);
    /*_iqref_temp = _iqref_temp - p_mpv->v.spest_radps * l_phys->Ke */    
    
    if(p_mpv->v.acc == ACC_POS)
    {
        if(p_mpv->v.spref_radps > 0.0f && _iqref_temp < 0.0f)
        {
            _iqref_temp = 0.0f;
            p_mpv->v.piSpeed.integ = l_iqControllerIntegPrev;
        }
        else if(p_mpv->v.spref_radps < 0.0f && _iqref_temp > 0.0f)
        {
            _iqref_temp = 0.0f;
            p_mpv->v.piSpeed.integ = l_iqControllerIntegPrev;
        }
    }
    p_mpv->v.iqref_spcontroller = _iqref_temp;
}

mcc_t FieldWeakening(const mcfwp_t *l_fwp, mcv_t l_vamp, mcv_t l_vbus, mcfwv_t *l_fwv)
{
    mcc_t idref_unsaturated;
    
    l_fwv->vmax = l_vbus * MC_SQRT3D3;
    l_fwv->vamp_filtered += l_fwv->vamp_lpfcoeff * (l_vamp - l_fwv->vamp_filtered);
    
    l_fwv->vmarg = l_fwp->vmaxcoeff * l_fwv->vmax - l_fwv->vamp_filtered;
    l_fwv->vmarg_filtered += l_fwv->vmarg_lpfcoeff * (l_fwv->vmarg - l_fwv->vmarg_filtered);
    idref_unsaturated = contDiscretePiController(l_fwv->vmarg_filtered, &l_fwv->pi);
    if(idref_unsaturated > (mcc_t)0.0f)
    {
        idref_unsaturated = (mcc_t)0.0f;
    }
    else
    {
        ;       /* do nothing */
    }

    if(l_fwv->pi.prop > 0.0f && l_fwv->pi.integ > 0.0f)
    {
        l_fwv->pi.integ = 0.0f;
    }
    else
    {
        ;       /* do nothing */
    }
    
    l_fwv->idref = idref_unsaturated;
        
    return l_fwv->idref;
}

void FieldWeakeningReset(mcfwv_t *l_fwv, mcv_t l_vamp)
{
    l_fwv->vamp_filtered = l_vamp;
    l_fwv->vmarg_filtered = 0.0f;
    l_fwv->pi.integ = 0.0f;
    l_fwv->idref = 0.0f;
}

static void mcEstimateSpeedAndPosition(mcCpv_t* p_mpv)
{
    mcdeg_t l_therr;

    if((p_mpv->v.statecom & FOC_RAMP_SPEED_ESTIMATION) != (mcstc_t)0)
    {
        mcObserversHandlerFL(p_mpv);

        if(p_mpv->v.state == RAMP)
        {
            p_mpv->v.thact = p_mpv->v.thest;
            p_mpv->v.spact = p_mpv->v.spest;
            
            p_mpv->v.thref = p_mpv->v.thact;
        }
        else
        {
            p_mpv->v.thact = p_mpv->v.thref;
            p_mpv->v.spact = p_mpv->v.spref;
        
            l_therr = p_mpv->v.thref.th -p_mpv->v.thest.th;
            if(l_therr < -(MC_PI2DEG))
            {
                p_mpv->v.therr = l_therr + MC_TWOPI2DEG;
        }
            else if(l_therr > MC_PI2DEG)
            {
                p_mpv->v.therr = l_therr - MC_TWOPI2DEG;
    }
        }
    }
    else
    {
        p_mpv->v.thact = p_mpv->v.thref;
        p_mpv->v.spact = p_mpv->v.spref;
        
        mcObserversInit(p_mpv);

        p_mpv->v.rbemf.spest = (mcsp_t)0;
        p_mpv->v.rbemf.thest.th = (mcdeg_t)0;
        p_mpv->v.rbemf.thest.cos = (mctri_t)1;
        p_mpv->v.rbemf.thest.sin = (mctri_t)0;
    }    
    p_mpv->v.sperr = p_mpv->v.spref - p_mpv->v.spest;

    p_mpv->v.spref_abs = mathAbsoluteFlt(p_mpv->v.spref);
    p_mpv->v.spact_abs = mathAbsoluteFlt(p_mpv->v.spact);
    p_mpv->v.sperr_abs = mathAbsoluteFlt(p_mpv->v.sperr);
    
    p_mpv->v.spref_radps = RPM2RADIAL(p_mpv->v.spref, p_mpv->p.phys.pp);
    p_mpv->v.spest_radps = RPM2RADIAL(p_mpv->v.spest, p_mpv->p.phys.pp);    
    p_mpv->v.spact_radps = RPM2RADIAL(p_mpv->v.spact, p_mpv->p.phys.pp);
}

void Modulation(mcCpv_t *p_mpv)
{
    if(p_mpv->v.spref_abs < p_mpv->p.modulation.speedR1)
    {
        if(p_mpv->p.modulation.typeR0 == MC_MODULATION_TYPE_SVPMW)
        {
            SpaceVectorModulation(&p_mpv->v.vabComp, &p_mpv->v.sector, &p_mpv->v.duvw);
        }
        else if(p_mpv->p.modulation.typeR0 == MC_MODULATION_TYPE_SVPMW_U0)
        {
            SpaceVectorModulationU0(&p_mpv->v.vabComp, &p_mpv->v.sector, &p_mpv->v.duvw);
        }
        else if(p_mpv->p.modulation.typeR0 == MC_MODULATION_TYPE_SVPMW_U7)
        {
            SpaceVectorModulationU7(&p_mpv->v.vabComp, &p_mpv->v.sector, &p_mpv->v.duvw);
        }
        else if(p_mpv->p.modulation.typeR0 == MC_MODULATION_TYPE_DPMW)
        {
            ;   /* not implemented yet */
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else
    {
        if(p_mpv->p.modulation.typeR1 == MC_MODULATION_TYPE_SVPMW)
        {
            SpaceVectorModulation(&p_mpv->v.vabComp, &p_mpv->v.sector, &p_mpv->v.duvw);
        }
        else if(p_mpv->p.modulation.typeR1 == MC_MODULATION_TYPE_SVPMW_U0)
        {
            SpaceVectorModulationU0(&p_mpv->v.vabComp, &p_mpv->v.sector, &p_mpv->v.duvw);
        }
        else if(p_mpv->p.modulation.typeR1 == MC_MODULATION_TYPE_SVPMW_U7)
        {
            SpaceVectorModulationU7(&p_mpv->v.vabComp, &p_mpv->v.sector, &p_mpv->v.duvw);
        }
        else if(p_mpv->p.modulation.typeR1 == MC_MODULATION_TYPE_DPMW)
        {
            ;   /* not implemented yet */
        }
        else
        {
            ;   /* do nothing */
        }
    }
}

static void Mtpa(mcCpv_t *p_mpv)
{
    mcmp_t id_calc1, id_calc2, id_calc3, id_calc4, id_calc5;
    mcc_t id_calc6;

    if(p_mpv->p.mtpa.enabled == MC_FALSE)
    {
        p_mpv->v.mtpa.idref = (mcc_t)0.0f;
        
        return;
    }
    else if((p_mpv->v.spest >= p_mpv->p.mtpa.minsp) && (p_mpv->v.spest < p_mpv->p.mtpa.maxsp))
    {
		id_calc1 = p_mpv->p.phys.Ke * p_mpv->p.phys.Ke;
		id_calc2 = 4.0f * p_mpv->v.diffLdLq * p_mpv->v.diffLdLq * p_mpv->v.idq.q * p_mpv->v.idq.q;
		id_calc3 = sqrtf(id_calc1 + id_calc2);
		id_calc4 = id_calc3 - p_mpv->p.phys.Ke;
		id_calc5 = id_calc4 * 0.5f;
		id_calc6 = id_calc5 / p_mpv->v.diffLdLq;
	}
    else
    {
        id_calc6 = (mcc_t)0.0f;
    }

    p_mpv->v.mtpa.idref += p_mpv->p.mtpa.lpfCoeff * (id_calc6 - p_mpv->v.mtpa.idref);
}

