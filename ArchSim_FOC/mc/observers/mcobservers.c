#include "mcinclude.h"
#include <math.h>

static void mcAngleTrackingObserverFL(mcv_ato_t *v, const mcp_ato_t *p);
static void mcRobustBemfObserverInit(mcCpv_t *p_mpv);
static void mcPiObsrvVarsReset(mcCpv_t *p_mpv);
static void mcRobustBemfObserverHandlerFL(mcCpv_t *p_mpv);
static void mcRobustBemfObserverHandlerSL(mcCpv_t *p_mpv);
static void mcEkfObserverInit(mcCpv_t *p_mpv);
static void mcEkfObserverHandlerFL(mcCpv_t *p_mpv);
static void mcEkfObsrvVarsReset(mcCpv_t *p_mpv);

static float LPF_FirstOrder(float vPrevFiltOut, float vFiltIn, float Ts, uint32_t fc)
{
    float vFiltOut = (vPrevFiltOut + Ts*2*MC_PI*fc*vFiltIn) / (1+Ts*2*MC_PI*fc);
    return vFiltOut;
}

void mcObserversInit(mcCpv_t *p_mpv)
{
    if(p_mpv->p.rampController == RAMP_CONTROLLER_RPIOBSERVER)
    {
        mcRobustBemfObserverInit(p_mpv);
    }
    else if(p_mpv->p.rampController == RAMP_CONTROLLER_EKFOBSERVER)
    {
    	mcEkfObserverInit(p_mpv);
    }
    else        /*  RAMP_CONTROLLER_NOT_SELECTED */
    {
       ;        /* do nothing */
    }
}
void mcObserversHandlerFL(mcCpv_t *p_mpv)
{
    if(p_mpv->p.rampController == RAMP_CONTROLLER_RPIOBSERVER)
    {
        mcRobustBemfObserverHandlerFL(p_mpv);
        p_mpv->v.spest = p_mpv->v.rbemf.spest;
        if(p_mpv->p.thest_offset == (mcdeg_t)0.0f)
        {
            p_mpv->v.thest = p_mpv->v.rbemf.thest;    
        }
        else
        {
            p_mpv->v.thest.th = contCheckAngle0to360deg((mc_float_t)(p_mpv->v.rbemf.thest.th + p_mpv->p.thest_offset));
            mathCalcSinCosFlt(p_mpv->v.thest.th, &p_mpv->v.thest.sin, &p_mpv->v.thest.cos);
        }
    }

    else if(p_mpv->p.rampController == RAMP_CONTROLLER_EKFOBSERVER)
    {
        mcEkfObserverHandlerFL(p_mpv);
        p_mpv->v.spest = p_mpv->v.ekf.speed_EKF;
        p_mpv->v.thest.th = (p_mpv->v.ekf.xk_inno_filt)*MC_RAD2DEG;
        p_mpv->v.thest.sin = sinf(p_mpv->v.ekf.xk_inno_filt);
        p_mpv->v.thest.cos = cosf(p_mpv->v.ekf.xk_inno_filt);
        //p_mpv->v.thest.sin = sin_array[(mc_int16_t)((p_mpv->v.ekf.xk_inno_filt)*MC_RAD2DEG*4.0f)];
        //p_mpv->v.thest.cos = cos_array[(mc_int16_t)((p_mpv->v.ekf.xk_inno_filt)*MC_RAD2DEG*4.0f)];

    }
    else        /*  RAMP_CONTROLLER_NOT_SELECTED */
    {
       ;        /* do nothing */
    }
}
void mcObserversHandlerSL(mcCpv_t *p_mpv)
{
    if(p_mpv->p.rampController == RAMP_CONTROLLER_RPIOBSERVER)
    {
        mcRobustBemfObserverHandlerSL(p_mpv);
    }
    else
    {
       ;        /* do nothing */
    }
}

static void mcAngleTrackingObserverFL(mcv_ato_t *v, const mcp_ato_t *p)
{
    v->w += v->k1 * v->in * v->dt;
    v->w = contSaturate(v->w,-10000.0f,10000.0f);
    
    v->wf += v->smc * (v->w - v->wf);
    v->integ += v->wf * v->dt;
    v->thrad = (p->k2 * v->wf) + v->integ;
    /* take into account circular nature of th_rad */    
    
    if(v->thrad >= (mcrad_t)MC_TWOPI)
    {
        v->thrad -= (mcrad_t)MC_TWOPI;
        v->integ -= (mcrad_t)MC_TWOPI;
    }
    else if(v->thrad < (mcrad_t)0.0f)
    {
        v->thrad += (mcrad_t)MC_TWOPI;
        v->integ += (mcrad_t)MC_TWOPI;
    }
    else
    {
        ;  /* do nothing */
    }
}

static void mcRobustBemfObserverInit(mcCpv_t *p_mpv)
{
    p_mpv->v.rbemf.egammaPiControllerVarId.kp = p_mpv->p.rbemf.egammaId.kp;
    p_mpv->v.rbemf.egammaPiControllerVarId.ki = p_mpv->p.rbemf.egammaId.ki / (mc_float_t)p_mpv->p.freqFL;
    p_mpv->v.rbemf.egammaPiControllerVarId.antiWindupCoef = p_mpv->p.rbemf.egammaId.antiWindupCoef;
    p_mpv->v.rbemf.egammaPiControllerVarId.antiWindupLimit = p_mpv->p.rbemf.egammaId.antiWindupLimit;
    p_mpv->v.rbemf.egammaPiControllerVarId.prop = (contPiPro_t)0.0f;
    p_mpv->v.rbemf.egammaPiControllerVarId.integ = (contPiInteg_t)0.0f;
    
    p_mpv->v.rbemf.egammaPiControllerVarIq.kp = p_mpv->p.rbemf.egammaIq.kp;
    p_mpv->v.rbemf.egammaPiControllerVarIq.ki = p_mpv->p.rbemf.egammaIq.ki / (mc_float_t)p_mpv->p.freqFL;
    p_mpv->v.rbemf.egammaPiControllerVarIq.antiWindupCoef = p_mpv->p.rbemf.egammaIq.antiWindupCoef;
    p_mpv->v.rbemf.egammaPiControllerVarIq.antiWindupLimit = p_mpv->p.rbemf.egammaIq.antiWindupLimit;
    p_mpv->v.rbemf.egammaPiControllerVarIq.prop = (contPiPro_t)0.0f;
    p_mpv->v.rbemf.egammaPiControllerVarIq.integ = (contPiInteg_t)0.0f;
    
    p_mpv->v.rbemf.edeltaPiControllerVarId.kp = p_mpv->p.rbemf.edeltaId.kp;
    p_mpv->v.rbemf.edeltaPiControllerVarId.ki = p_mpv->p.rbemf.edeltaId.ki / (mc_float_t)p_mpv->p.freqFL;
    p_mpv->v.rbemf.edeltaPiControllerVarId.antiWindupCoef = p_mpv->p.rbemf.edeltaId.antiWindupCoef;
    p_mpv->v.rbemf.edeltaPiControllerVarId.antiWindupLimit = p_mpv->p.rbemf.edeltaId.antiWindupLimit;
    p_mpv->v.rbemf.edeltaPiControllerVarId.prop = (contPiPro_t)0.0f;
    p_mpv->v.rbemf.edeltaPiControllerVarId.integ = (contPiInteg_t)0.0f;
    
    p_mpv->v.rbemf.edeltaPiControllerVarIq.kp = p_mpv->p.rbemf.edeltaIq.kp;
    p_mpv->v.rbemf.edeltaPiControllerVarIq.ki = p_mpv->p.rbemf.edeltaIq.ki / (mc_float_t)p_mpv->p.freqFL;
    p_mpv->v.rbemf.edeltaPiControllerVarIq.antiWindupCoef = p_mpv->p.rbemf.edeltaIq.antiWindupCoef;
    p_mpv->v.rbemf.edeltaPiControllerVarIq.antiWindupLimit = p_mpv->p.rbemf.edeltaIq.antiWindupLimit;
    p_mpv->v.rbemf.edeltaPiControllerVarIq.prop = (contPiPro_t)0.0f;
    p_mpv->v.rbemf.edeltaPiControllerVarIq.integ = (contPiInteg_t)0.0f;

    p_mpv->v.rbemf.tsamp = p_mpv->v.periodFL;
    
    p_mpv->v.rbemf.invld = 1.0f / p_mpv->p.phys.Ld;
    p_mpv->v.rbemf.invlq = 1.0f / p_mpv->p.phys.Lq;
    
    if(p_mpv->v.rbemf.lpfcoeff < (mc_float_t)p_mpv->p.freqFL)
    {
        p_mpv->v.rbemf.lpfcoeff = p_mpv->p.rbemf.lpfcoeff / (mc_float_t)p_mpv->p.freqFL;
    }
    else
    {
        p_mpv->v.rbemf.lpfcoeff = MC_FLOAT32_VALUE_ONE_LESS;
    }

    p_mpv->v.ato.dt = p_mpv->v.periodFL;
    p_mpv->v.ato.k1_coeff = (p_mpv->p.ato.k1max - p_mpv->p.ato.k1min) / 
                            (p_mpv->p.sprefmax - p_mpv->p.minsp_o2c);
    if(p_mpv->p.ato.smc < (mc_float_t)p_mpv->p.freqFL)
    {
        p_mpv->v.ato.smc = p_mpv->p.ato.smc / (mc_float_t)p_mpv->p.freqFL;
    }
    else
    {
        p_mpv->v.ato.smc = MC_FLOAT32_VALUE_ONE_LESS;
    }
    if(p_mpv->p.ato.thrad_lpfcoeff < (mc_float_t)p_mpv->p.freqFL)
    {
        p_mpv->v.ato.thrad_lpfcoeff = p_mpv->p.ato.thrad_lpfcoeff / (mc_float_t)p_mpv->p.freqFL;
    }
    else
    {
        p_mpv->v.ato.thrad_lpfcoeff = MC_FLOAT32_VALUE_ONE_LESS;
    }
        
    p_mpv->v.rbemf.loop_cntr = 0u;
    
    mcPiObsrvVarsReset(p_mpv);
}

static void mcEkfObserverInit(mcCpv_t *p_mpv)
{
    p_mpv->v.ekf.I_EKF[0][0] = 1.0f;
    p_mpv->v.ekf.I_EKF[1][1] = 1.0f;
    p_mpv->v.ekf.I_EKF[2][2] = 1.0f;
    p_mpv->v.ekf.I_EKF[3][3] = 1.0f;

    p_mpv->v.ekf.Jh[0][0] = 1.0f;
    p_mpv->v.ekf.Jh[0][1] = 0.0f;
    p_mpv->v.ekf.Jh[0][2] = 0.0f;
    p_mpv->v.ekf.Jh[0][3] = 0.0f;
    p_mpv->v.ekf.Jh[1][0] = 0.0f;
    p_mpv->v.ekf.Jh[1][1] = 1.0f;
    p_mpv->v.ekf.Jh[1][2] = 0.0f;
    p_mpv->v.ekf.Jh[1][3] = 0.0f;

    p_mpv->v.ekf.Jh_trans[0][0] = 1.0f;
    p_mpv->v.ekf.Jh_trans[0][1] = 0.0f;
    p_mpv->v.ekf.Jh_trans[1][0] = 0.0f;
    p_mpv->v.ekf.Jh_trans[1][1] = 1.0f;
    p_mpv->v.ekf.Jh_trans[2][0] = 0.0f;
    p_mpv->v.ekf.Jh_trans[2][1] = 0.0f;
    p_mpv->v.ekf.Jh_trans[3][0] = 0.0f;
    p_mpv->v.ekf.Jh_trans[3][1] = 0.0f;

    p_mpv->v.ekf.xk_est[0][0] = 0.0f;
    p_mpv->v.ekf.xk_est[1][0] = 0.0f;
    p_mpv->v.ekf.xk_est[2][0] = 0.0f;
    p_mpv->v.ekf.xk_est[3][0] = 0.0f;

    p_mpv->v.ekf.Rk_real[0][0] = p_mpv->p.ekf.Rk[0][0];
    p_mpv->v.ekf.Rk_real[1][1] = p_mpv->p.ekf.Rk[0][0];

    p_mpv->v.ekf.Qk_real[0][0] = p_mpv->p.ekf.Qk_ol[0][0];
    p_mpv->v.ekf.Qk_real[1][1] = p_mpv->p.ekf.Qk_ol[0][0];
    p_mpv->v.ekf.Qk_real[2][2] = 1.6f;
    p_mpv->v.ekf.Qk_real[3][3] = p_mpv->p.ekf.Qk_ol[3][3];

    p_mpv->v.ekf.Pk_inno_old[0][0] = 100.0f;
    p_mpv->v.ekf.Pk_inno_old[1][1] = 100.0f;
    p_mpv->v.ekf.Pk_inno_old[2][2] = 100.0f;
    p_mpv->v.ekf.Pk_inno_old[3][3] = 100.0f;

    p_mpv->v.ekf.Rs = p_mpv->p.phys.r;
    p_mpv->v.ekf.L0 = (p_mpv->p.phys.Ld + p_mpv->p.phys.Lq)/2.0f;
    p_mpv->v.ekf.theta_EKF_Shift_real =0.0f;

    mcEkfObsrvVarsReset(p_mpv);
}
static void mcPiObsrvVarsReset(mcCpv_t *p_mpv)
{
    p_mpv->v.ato.in = 0.0f;
    p_mpv->v.ato.in_prev = 0.0f;
    p_mpv->v.ato.w = 0.0f;
    p_mpv->v.ato.wf = 0.0f;
    p_mpv->v.ato.integ = 0.0f;
    p_mpv->v.ato.thradfiltered = 0.0f;
    
    p_mpv->v.rbemf.edelta = 0.0f;
    p_mpv->v.rbemf.egamma = 0.0f;
    p_mpv->v.rbemf.idqest.d = 0.0f;
    p_mpv->v.rbemf.idqest.q = 0.0f;
    p_mpv->v.rbemf.egammaPiControllerVarId.integ = 0.0f;
    p_mpv->v.rbemf.egammaPiControllerVarIq.integ = 0.0f;
    p_mpv->v.rbemf.edeltaPiControllerVarId.integ = 0.0f;
    p_mpv->v.rbemf.edeltaPiControllerVarIq.integ = 0.0f;
    p_mpv->v.rbemf.idqref.d = 0.0f;
    p_mpv->v.rbemf.idqref.q = 0.0f;
}

static void mcEkfObsrvVarsReset(mcCpv_t *p_mpv)
{
    p_mpv->v.ekf.I_EKF[0][0] = 1.0f;
    p_mpv->v.ekf.I_EKF[1][1] = 1.0f;
    p_mpv->v.ekf.I_EKF[2][2] = 1.0f;
    p_mpv->v.ekf.I_EKF[3][3] = 1.0f;

    p_mpv->v.ekf.Jh[0][0] = 1.0f;
    p_mpv->v.ekf.Jh[0][1] = 0.0f;
    p_mpv->v.ekf.Jh[0][2] = 0.0f;
    p_mpv->v.ekf.Jh[0][3] = 0.0f;
    p_mpv->v.ekf.Jh[1][0] = 0.0f;
    p_mpv->v.ekf.Jh[1][1] = 1.0f;
    p_mpv->v.ekf.Jh[1][2] = 0.0f;
    p_mpv->v.ekf.Jh[1][3] = 0.0f;

    p_mpv->v.ekf.Jh_trans[0][0] = 1.0f;
    p_mpv->v.ekf.Jh_trans[0][1] = 0.0f;
    p_mpv->v.ekf.Jh_trans[1][0] = 0.0f;
    p_mpv->v.ekf.Jh_trans[1][1] = 1.0f;
    p_mpv->v.ekf.Jh_trans[2][0] = 0.0f;
    p_mpv->v.ekf.Jh_trans[2][1] = 0.0f;
    p_mpv->v.ekf.Jh_trans[3][0] = 0.0f;
    p_mpv->v.ekf.Jh_trans[3][1] = 0.0f;

    p_mpv->v.ekf.xk_est[0][0] = 0.0f;
    p_mpv->v.ekf.xk_est[1][0] = 0.0f;
    p_mpv->v.ekf.xk_est[2][0] = 0.0f;
    p_mpv->v.ekf.xk_est[3][0] = 0.0f;

    p_mpv->v.ekf.Rk_real[0][0] = p_mpv->p.ekf.Rk[0][0];
    p_mpv->v.ekf.Rk_real[1][1] = p_mpv->p.ekf.Rk[0][0];

    p_mpv->v.ekf.Qk_real[0][0] = p_mpv->p.ekf.Qk_ol[0][0];
    p_mpv->v.ekf.Qk_real[1][1] = p_mpv->p.ekf.Qk_ol[0][0];
    p_mpv->v.ekf.Qk_real[2][2] = 1.6f;
    p_mpv->v.ekf.Qk_real[3][3] = p_mpv->p.ekf.Qk_ol[3][3];

    p_mpv->v.ekf.Pk_inno_old[0][0] = 100.0f;
    p_mpv->v.ekf.Pk_inno_old[1][1] = 100.0f;
    p_mpv->v.ekf.Pk_inno_old[2][2] = 100.0f;
    p_mpv->v.ekf.Pk_inno_old[3][3] = 100.0f;


    p_mpv->v.ekf.theta_EKF_Shift_real =0.0f;

}

static void mcRobustBemfObserverHandlerFL(mcCpv_t *p_mpv)
{
    mcv_t egammaId, egammaIq, edeltaId, edeltaIq;
    mcv_t egamma, edelta;

    ParkTransform(&p_mpv->v.vab, &p_mpv->v.rbemf.thest, &p_mpv->v.rbemf.vdq);
    ParkTransform(&p_mpv->v.iab, &p_mpv->v.rbemf.thest, &p_mpv->v.rbemf.idqref);

    p_mpv->v.rbemf.iderr = p_mpv->v.rbemf.idqref.d - p_mpv->v.rbemf.idqest.d;
    p_mpv->v.rbemf.iqerr = p_mpv->v.rbemf.idqref.q - p_mpv->v.rbemf.idqest.q;
    
    egammaId = contDiscretePiController(p_mpv->v.rbemf.iderr, &p_mpv->v.rbemf.egammaPiControllerVarId);
    egammaIq = contDiscretePiController(p_mpv->v.rbemf.iqerr, &p_mpv->v.rbemf.egammaPiControllerVarIq);
    edeltaId = contDiscretePiController(p_mpv->v.rbemf.iderr, &p_mpv->v.rbemf.edeltaPiControllerVarId);
    edeltaIq = contDiscretePiController(p_mpv->v.rbemf.iqerr, &p_mpv->v.rbemf.edeltaPiControllerVarIq);

    egamma = p_mpv->v.rbemf.egamma;
    edelta = p_mpv->v.rbemf.edelta;
    
    egamma += p_mpv->v.rbemf.lpfcoeff * (egammaId + egammaIq - egamma);
    edelta += p_mpv->v.rbemf.lpfcoeff * (edeltaId + edeltaIq - edelta);
    
    p_mpv->v.rbemf.egamma = contSaturate(egamma, -p_mpv->p.rbemf.egammaMax, p_mpv->p.rbemf.egammaMax);
    p_mpv->v.rbemf.edelta = contSaturate(edelta, -p_mpv->p.rbemf.edeltaMax, p_mpv->p.rbemf.edeltaMax);

    p_mpv->v.rbemf.idqest.d = p_mpv->v.rbemf.tsamp * p_mpv->v.rbemf.invld * (p_mpv->v.rbemf.vdq.d + (p_mpv->p.phys.Ld / p_mpv->v.rbemf.tsamp - p_mpv->p.phys.r) * p_mpv->v.rbemf.idqref.d + 
                                                  p_mpv->p.phys.Lq * p_mpv->v.ato.wf * p_mpv->v.rbemf.idqref.q - p_mpv->v.rbemf.egamma);
    p_mpv->v.rbemf.idqest.q = p_mpv->v.rbemf.tsamp * p_mpv->v.rbemf.invlq * (p_mpv->v.rbemf.vdq.q + (p_mpv->p.phys.Lq / p_mpv->v.rbemf.tsamp - p_mpv->p.phys.r) * p_mpv->v.rbemf.idqref.q - 
                                                  p_mpv->p.phys.Ld * p_mpv->v.ato.wf * p_mpv->v.rbemf.idqref.d - p_mpv->v.rbemf.edelta);
        
    if(((p_mpv->v.spref < p_mpv->p.rbemf.minsp) && (p_mpv->v.spref > -p_mpv->p.rbemf.minsp)) ||
       ((p_mpv->v.rbemf.edelta < p_mpv->p.rbemf.edeltaMin) && (p_mpv->v.rbemf.edelta > -p_mpv->p.rbemf.edeltaMin)))
    {
        mcPiObsrvVarsReset(p_mpv);
        
        egamma = edelta = 0.0f;
    }
    else
    {
        if(p_mpv->v.rot == ROT_CW)
        {
            /* p_mpv->v.ato.in = MC_PI * mathArcTanFlt(-p_mpv->v.rbemf.egamma, p_mpv->v.rbemf.edelta) / 180.0f */
            p_mpv->v.ato.in = atan2f(-p_mpv->v.rbemf.egamma, p_mpv->v.rbemf.edelta);
        }
        else if(p_mpv->v.rot == ROT_CCW)
        {
            /* p_mpv->v.ato.in = MC_PI * mathArcTanFlt(p_mpv->v.rbemf.egamma, -p_mpv->v.rbemf.edelta) / 180.0f */
            p_mpv->v.ato.in = atan2f(p_mpv->v.rbemf.egamma, -p_mpv->v.rbemf.edelta);
        }
        else
        {
            ;   /* do nothing */
        }
        mcAngleTrackingObserverFL(&p_mpv->v.ato, &p_mpv->p.ato);                     
        p_mpv->v.ato.thradfiltered += p_mpv->v.ato.thrad_lpfcoeff*(p_mpv->v.ato.thrad - p_mpv->v.ato.thradfiltered);
        if(p_mpv->v.ato.thradfiltered >= MC_TWOPI)
        {
            p_mpv->v.ato.thradfiltered -= MC_TWOPI;
        }
        else if(p_mpv->v.ato.thradfiltered < 0.0f)
        {
            p_mpv->v.ato.thradfiltered += MC_TWOPI;
        }
        else
        {
            ;   /* do nothing */
        }
    }

    p_mpv->v.rbemf.spest = ((mcsp_t)MC_RF2RPM) * p_mpv->v.ato.wf / (mcsp_t)p_mpv->p.phys.pp;
    p_mpv->v.rbemf.thest.th = MC_RAD2DEG * p_mpv->v.ato.thradfiltered;
    mathCalcSinCosFlt(p_mpv->v.rbemf.thest.th, &p_mpv->v.rbemf.thest.sin, &p_mpv->v.rbemf.thest.cos);
    
    p_mpv->v.rbemf.loop_cntr++;
}

static void mcEkfObserverHandlerFL(mcCpv_t *p_mpv)
{

    p_mpv->v.ekf.Rs = p_mpv->p.phys.r;
    p_mpv->v.ekf.L0 = (p_mpv->p.phys.Ld + p_mpv->p.phys.Lq)/2.0f;

    p_mpv->v.ekf.Vku_old[0][0] = p_mpv->v.vab.a;
    p_mpv->v.ekf.Vku_old[1][0] = p_mpv->v.vab.b;

    p_mpv->v.ekf.xk_est[0][0] = p_mpv->v.ekf.xk_inno_old[0][0] + (-(p_mpv->v.ekf.Rs)*p_mpv->v.ekf.xk_inno_old[0][0] + p_mpv->v.ekf.xk_inno_old[2][0]*p_mpv->p.phys.Phi*sinf(p_mpv->v.ekf.xk_inno_old[3][0]) + p_mpv->v.ekf.Vku_old[0][0])/p_mpv->v.ekf.L0/(p_mpv->p.freqFL);
    p_mpv->v.ekf.xk_est[1][0] = p_mpv->v.ekf.xk_inno_old[1][0] + (-(p_mpv->v.ekf.Rs)*p_mpv->v.ekf.xk_inno_old[1][0] - p_mpv->v.ekf.xk_inno_old[2][0]*p_mpv->p.phys.Phi*cosf(p_mpv->v.ekf.xk_inno_old[3][0]) + p_mpv->v.ekf.Vku_old[1][0])/p_mpv->v.ekf.L0/(p_mpv->p.freqFL);
    p_mpv->v.ekf.xk_est[2][0] = p_mpv->v.ekf.xk_inno_old[2][0];
    p_mpv->v.ekf.xk_est[3][0] = p_mpv->v.ekf.xk_inno_old[3][0] + p_mpv->v.ekf.xk_inno_old[2][0]/(p_mpv->p.freqFL);

    p_mpv->v.ekf.Jf[0][0] = 1.0f-(p_mpv->v.ekf.Rs)/(p_mpv->p.freqFL)/p_mpv->v.ekf.L0;
    p_mpv->v.ekf.Jf[0][1] = 0.0f;
    p_mpv->v.ekf.Jf[0][2] = p_mpv->p.phys.Phi/(p_mpv->p.freqFL)/p_mpv->v.ekf.L0*sinf(p_mpv->v.ekf.xk_inno_old[3][0]);
    p_mpv->v.ekf.Jf[0][3] = p_mpv->v.ekf.xk_inno_old[2][0]*p_mpv->p.phys.Phi*cosf(p_mpv->v.ekf.xk_inno_old[3][0])/(p_mpv->p.freqFL)/p_mpv->v.ekf.L0;
    p_mpv->v.ekf.Jf[1][0] = 0.0f;
    p_mpv->v.ekf.Jf[1][1] = p_mpv->v.ekf.Jf[0][0];
    p_mpv->v.ekf.Jf[1][2] = -p_mpv->p.phys.Phi/(p_mpv->p.freqFL)/p_mpv->v.ekf.L0*cosf(p_mpv->v.ekf.xk_inno_old[3][0]);
    p_mpv->v.ekf.Jf[1][3] = p_mpv->v.ekf.xk_inno_old[2][0]*p_mpv->p.phys.Phi*sinf(p_mpv->v.ekf.xk_inno_old[3][0])/(p_mpv->p.freqFL)/p_mpv->v.ekf.L0;
    p_mpv->v.ekf.Jf[2][0] = 0.0f;
    p_mpv->v.ekf.Jf[2][1] = 0.0f;
    p_mpv->v.ekf.Jf[2][2] = 1.0f;
    p_mpv->v.ekf.Jf[2][3] = 0.0f;
    p_mpv->v.ekf.Jf[3][0] = 0.0f;
    p_mpv->v.ekf.Jf[3][1] = 0.0f;
    p_mpv->v.ekf.Jf[3][2] = 1.0f/(p_mpv->p.freqFL);
    p_mpv->v.ekf.Jf[3][3] = 1.0f;

    p_mpv->v.ekf.Jf_trans[0][0] = p_mpv->v.ekf.Jf[0][0];
    p_mpv->v.ekf.Jf_trans[1][0] = p_mpv->v.ekf.Jf[0][1];
    p_mpv->v.ekf.Jf_trans[2][0] = p_mpv->v.ekf.Jf[0][2];
    p_mpv->v.ekf.Jf_trans[3][0] = p_mpv->v.ekf.Jf[0][3];
    p_mpv->v.ekf.Jf_trans[0][1] = p_mpv->v.ekf.Jf[1][0];
    p_mpv->v.ekf.Jf_trans[1][1] = p_mpv->v.ekf.Jf[1][1];
    p_mpv->v.ekf.Jf_trans[2][1] = p_mpv->v.ekf.Jf[1][2];
    p_mpv->v.ekf.Jf_trans[3][1] = p_mpv->v.ekf.Jf[1][3];
    p_mpv->v.ekf.Jf_trans[0][2] = p_mpv->v.ekf.Jf[2][0];
    p_mpv->v.ekf.Jf_trans[1][2] = p_mpv->v.ekf.Jf[2][1];
    p_mpv->v.ekf.Jf_trans[2][2] = p_mpv->v.ekf.Jf[2][2];
    p_mpv->v.ekf.Jf_trans[3][2] = p_mpv->v.ekf.Jf[2][3];
    p_mpv->v.ekf.Jf_trans[0][3] = p_mpv->v.ekf.Jf[3][0];
    p_mpv->v.ekf.Jf_trans[1][3] = p_mpv->v.ekf.Jf[3][1];
    p_mpv->v.ekf.Jf_trans[2][3] = p_mpv->v.ekf.Jf[3][2];
    p_mpv->v.ekf.Jf_trans[3][3] = p_mpv->v.ekf.Jf[3][3];

    p_mpv->v.ekf.zk[0][0] = p_mpv->v.iab.a;
    p_mpv->v.ekf.zk[1][0] = p_mpv->v.iab.b;

    p_mpv->v.ekf.hxk[0][0] = p_mpv->v.ekf.xk_est[0][0];
    p_mpv->v.ekf.hxk[1][0] = p_mpv->v.ekf.xk_est[1][0];

    p_mpv->v.ekf.multi_Jf_Pkinnoold[0][0] = p_mpv->v.ekf.Jf[0][0]*p_mpv->v.ekf.Pk_inno_old[0][0]+p_mpv->v.ekf.Jf[0][1]*p_mpv->v.ekf.Pk_inno_old[1][0]+p_mpv->v.ekf.Jf[0][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.Jf[0][3]*p_mpv->v.ekf.Pk_inno_old[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[0][1] = p_mpv->v.ekf.Jf[0][0]*p_mpv->v.ekf.Pk_inno_old[0][1]+p_mpv->v.ekf.Jf[0][1]*p_mpv->v.ekf.Pk_inno_old[1][1]+p_mpv->v.ekf.Jf[0][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.Jf[0][3]*p_mpv->v.ekf.Pk_inno_old[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[0][2] = p_mpv->v.ekf.Jf[0][0]*p_mpv->v.ekf.Pk_inno_old[0][2]+p_mpv->v.ekf.Jf[0][1]*p_mpv->v.ekf.Pk_inno_old[1][2]+p_mpv->v.ekf.Jf[0][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.Jf[0][3]*p_mpv->v.ekf.Pk_inno_old[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[0][3] = p_mpv->v.ekf.Jf[0][0]*p_mpv->v.ekf.Pk_inno_old[0][3]+p_mpv->v.ekf.Jf[0][1]*p_mpv->v.ekf.Pk_inno_old[1][3]+p_mpv->v.ekf.Jf[0][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.Jf[0][3]*p_mpv->v.ekf.Pk_inno_old[3][3];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[1][0] = p_mpv->v.ekf.Jf[1][0]*p_mpv->v.ekf.Pk_inno_old[0][0]+p_mpv->v.ekf.Jf[1][1]*p_mpv->v.ekf.Pk_inno_old[1][0]+p_mpv->v.ekf.Jf[1][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.Jf[1][3]*p_mpv->v.ekf.Pk_inno_old[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[1][1] = p_mpv->v.ekf.Jf[1][0]*p_mpv->v.ekf.Pk_inno_old[0][1]+p_mpv->v.ekf.Jf[1][1]*p_mpv->v.ekf.Pk_inno_old[1][1]+p_mpv->v.ekf.Jf[1][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.Jf[1][3]*p_mpv->v.ekf.Pk_inno_old[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[1][2] = p_mpv->v.ekf.Jf[1][0]*p_mpv->v.ekf.Pk_inno_old[0][2]+p_mpv->v.ekf.Jf[1][1]*p_mpv->v.ekf.Pk_inno_old[1][2]+p_mpv->v.ekf.Jf[1][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.Jf[1][3]*p_mpv->v.ekf.Pk_inno_old[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[1][3] = p_mpv->v.ekf.Jf[1][0]*p_mpv->v.ekf.Pk_inno_old[0][3]+p_mpv->v.ekf.Jf[1][1]*p_mpv->v.ekf.Pk_inno_old[1][3]+p_mpv->v.ekf.Jf[1][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.Jf[1][3]*p_mpv->v.ekf.Pk_inno_old[3][3];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[2][0] = p_mpv->v.ekf.Jf[2][0]*p_mpv->v.ekf.Pk_inno_old[0][0]+p_mpv->v.ekf.Jf[2][1]*p_mpv->v.ekf.Pk_inno_old[1][0]+p_mpv->v.ekf.Jf[2][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.Jf[2][3]*p_mpv->v.ekf.Pk_inno_old[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[2][1] = p_mpv->v.ekf.Jf[2][0]*p_mpv->v.ekf.Pk_inno_old[0][1]+p_mpv->v.ekf.Jf[2][1]*p_mpv->v.ekf.Pk_inno_old[1][1]+p_mpv->v.ekf.Jf[2][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.Jf[2][3]*p_mpv->v.ekf.Pk_inno_old[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[2][2] = p_mpv->v.ekf.Jf[2][0]*p_mpv->v.ekf.Pk_inno_old[0][2]+p_mpv->v.ekf.Jf[2][1]*p_mpv->v.ekf.Pk_inno_old[1][2]+p_mpv->v.ekf.Jf[2][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.Jf[2][3]*p_mpv->v.ekf.Pk_inno_old[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[2][3] = p_mpv->v.ekf.Jf[2][0]*p_mpv->v.ekf.Pk_inno_old[0][3]+p_mpv->v.ekf.Jf[2][1]*p_mpv->v.ekf.Pk_inno_old[1][3]+p_mpv->v.ekf.Jf[2][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.Jf[2][3]*p_mpv->v.ekf.Pk_inno_old[3][3];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[3][0] = p_mpv->v.ekf.Jf[3][0]*p_mpv->v.ekf.Pk_inno_old[0][0]+p_mpv->v.ekf.Jf[3][1]*p_mpv->v.ekf.Pk_inno_old[1][0]+p_mpv->v.ekf.Jf[3][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.Jf[3][3]*p_mpv->v.ekf.Pk_inno_old[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[3][1] = p_mpv->v.ekf.Jf[3][0]*p_mpv->v.ekf.Pk_inno_old[0][1]+p_mpv->v.ekf.Jf[3][1]*p_mpv->v.ekf.Pk_inno_old[1][1]+p_mpv->v.ekf.Jf[3][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.Jf[3][3]*p_mpv->v.ekf.Pk_inno_old[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[3][2] = p_mpv->v.ekf.Jf[3][0]*p_mpv->v.ekf.Pk_inno_old[0][2]+p_mpv->v.ekf.Jf[3][1]*p_mpv->v.ekf.Pk_inno_old[1][2]+p_mpv->v.ekf.Jf[3][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.Jf[3][3]*p_mpv->v.ekf.Pk_inno_old[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold[3][3] = p_mpv->v.ekf.Jf[3][0]*p_mpv->v.ekf.Pk_inno_old[0][3]+p_mpv->v.ekf.Jf[3][1]*p_mpv->v.ekf.Pk_inno_old[1][3]+p_mpv->v.ekf.Jf[3][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.Jf[3][3]*p_mpv->v.ekf.Pk_inno_old[3][3];

    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold[0][0]*p_mpv->v.ekf.Jf_trans[0][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][1]*p_mpv->v.ekf.Jf_trans[1][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][2]*p_mpv->v.ekf.Jf_trans[2][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][3]*p_mpv->v.ekf.Jf_trans[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold[0][0]*p_mpv->v.ekf.Jf_trans[0][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][1]*p_mpv->v.ekf.Jf_trans[1][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][2]*p_mpv->v.ekf.Jf_trans[2][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][3]*p_mpv->v.ekf.Jf_trans[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold[0][0]*p_mpv->v.ekf.Jf_trans[0][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][1]*p_mpv->v.ekf.Jf_trans[1][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][2]*p_mpv->v.ekf.Jf_trans[2][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][3]*p_mpv->v.ekf.Jf_trans[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold[0][0]*p_mpv->v.ekf.Jf_trans[0][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][1]*p_mpv->v.ekf.Jf_trans[1][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][2]*p_mpv->v.ekf.Jf_trans[2][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[0][3]*p_mpv->v.ekf.Jf_trans[3][3];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold[1][0]*p_mpv->v.ekf.Jf_trans[0][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][1]*p_mpv->v.ekf.Jf_trans[1][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][2]*p_mpv->v.ekf.Jf_trans[2][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][3]*p_mpv->v.ekf.Jf_trans[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold[1][0]*p_mpv->v.ekf.Jf_trans[0][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][1]*p_mpv->v.ekf.Jf_trans[1][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][2]*p_mpv->v.ekf.Jf_trans[2][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][3]*p_mpv->v.ekf.Jf_trans[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold[1][0]*p_mpv->v.ekf.Jf_trans[0][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][1]*p_mpv->v.ekf.Jf_trans[1][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][2]*p_mpv->v.ekf.Jf_trans[2][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][3]*p_mpv->v.ekf.Jf_trans[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold[1][0]*p_mpv->v.ekf.Jf_trans[0][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][1]*p_mpv->v.ekf.Jf_trans[1][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][2]*p_mpv->v.ekf.Jf_trans[2][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[1][3]*p_mpv->v.ekf.Jf_trans[3][3];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold[2][0]*p_mpv->v.ekf.Jf_trans[0][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][1]*p_mpv->v.ekf.Jf_trans[1][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][2]*p_mpv->v.ekf.Jf_trans[2][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][3]*p_mpv->v.ekf.Jf_trans[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold[2][0]*p_mpv->v.ekf.Jf_trans[0][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][1]*p_mpv->v.ekf.Jf_trans[1][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][2]*p_mpv->v.ekf.Jf_trans[2][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][3]*p_mpv->v.ekf.Jf_trans[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold[2][0]*p_mpv->v.ekf.Jf_trans[0][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][1]*p_mpv->v.ekf.Jf_trans[1][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][2]*p_mpv->v.ekf.Jf_trans[2][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][3]*p_mpv->v.ekf.Jf_trans[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold[2][0]*p_mpv->v.ekf.Jf_trans[0][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][1]*p_mpv->v.ekf.Jf_trans[1][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][2]*p_mpv->v.ekf.Jf_trans[2][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[2][3]*p_mpv->v.ekf.Jf_trans[3][3];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold[3][0]*p_mpv->v.ekf.Jf_trans[0][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][1]*p_mpv->v.ekf.Jf_trans[1][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][2]*p_mpv->v.ekf.Jf_trans[2][0]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][3]*p_mpv->v.ekf.Jf_trans[3][0];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold[3][0]*p_mpv->v.ekf.Jf_trans[0][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][1]*p_mpv->v.ekf.Jf_trans[1][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][2]*p_mpv->v.ekf.Jf_trans[2][1]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][3]*p_mpv->v.ekf.Jf_trans[3][1];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold[3][0]*p_mpv->v.ekf.Jf_trans[0][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][1]*p_mpv->v.ekf.Jf_trans[1][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][2]*p_mpv->v.ekf.Jf_trans[2][2]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][3]*p_mpv->v.ekf.Jf_trans[3][2];
    p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold[3][0]*p_mpv->v.ekf.Jf_trans[0][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][1]*p_mpv->v.ekf.Jf_trans[1][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][2]*p_mpv->v.ekf.Jf_trans[2][3]+p_mpv->v.ekf.multi_Jf_Pkinnoold[3][3]*p_mpv->v.ekf.Jf_trans[3][3];

    p_mpv->v.ekf.Pk_est[0][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][0]+p_mpv->v.ekf.Qk_real[0][0];
    p_mpv->v.ekf.Pk_est[0][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][1]+p_mpv->v.ekf.Qk_real[0][1];
    p_mpv->v.ekf.Pk_est[0][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][2]+p_mpv->v.ekf.Qk_real[0][2];
    p_mpv->v.ekf.Pk_est[0][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[0][3]+p_mpv->v.ekf.Qk_real[0][3];
    p_mpv->v.ekf.Pk_est[1][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][0]+p_mpv->v.ekf.Qk_real[1][0];
    p_mpv->v.ekf.Pk_est[1][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][1]+p_mpv->v.ekf.Qk_real[1][1];
    p_mpv->v.ekf.Pk_est[1][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][2]+p_mpv->v.ekf.Qk_real[1][2];
    p_mpv->v.ekf.Pk_est[1][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[1][3]+p_mpv->v.ekf.Qk_real[1][3];
    p_mpv->v.ekf.Pk_est[2][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][0]+p_mpv->v.ekf.Qk_real[2][0];
    p_mpv->v.ekf.Pk_est[2][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][1]+p_mpv->v.ekf.Qk_real[2][1];
    p_mpv->v.ekf.Pk_est[2][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][2]+p_mpv->v.ekf.Qk_real[2][2];
    p_mpv->v.ekf.Pk_est[2][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[2][3]+p_mpv->v.ekf.Qk_real[2][3];
    p_mpv->v.ekf.Pk_est[3][0] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][0]+p_mpv->v.ekf.Qk_real[3][0];
    p_mpv->v.ekf.Pk_est[3][1] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][1]+p_mpv->v.ekf.Qk_real[3][1];
    p_mpv->v.ekf.Pk_est[3][2] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][2]+p_mpv->v.ekf.Qk_real[3][2];
    p_mpv->v.ekf.Pk_est[3][3] = p_mpv->v.ekf.multi_Jf_Pkinnoold_Jftrans[3][3]+p_mpv->v.ekf.Qk_real[3][3];

    p_mpv->v.ekf.multi_Pkest_Jhtrans[0][0] = p_mpv->v.ekf.Pk_est[0][0]*p_mpv->v.ekf.Jh_trans[0][0]+p_mpv->v.ekf.Pk_est[0][1]*p_mpv->v.ekf.Jh_trans[1][0]+p_mpv->v.ekf.Pk_est[0][2]*p_mpv->v.ekf.Jh_trans[2][0]+p_mpv->v.ekf.Pk_est[0][3]*p_mpv->v.ekf.Jh_trans[3][0];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[0][1] = p_mpv->v.ekf.Pk_est[0][0]*p_mpv->v.ekf.Jh_trans[0][1]+p_mpv->v.ekf.Pk_est[0][1]*p_mpv->v.ekf.Jh_trans[1][1]+p_mpv->v.ekf.Pk_est[0][2]*p_mpv->v.ekf.Jh_trans[2][1]+p_mpv->v.ekf.Pk_est[0][3]*p_mpv->v.ekf.Jh_trans[3][1];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[1][0] = p_mpv->v.ekf.Pk_est[1][0]*p_mpv->v.ekf.Jh_trans[0][0]+p_mpv->v.ekf.Pk_est[1][1]*p_mpv->v.ekf.Jh_trans[1][0]+p_mpv->v.ekf.Pk_est[1][2]*p_mpv->v.ekf.Jh_trans[2][0]+p_mpv->v.ekf.Pk_est[1][3]*p_mpv->v.ekf.Jh_trans[3][0];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[1][1] = p_mpv->v.ekf.Pk_est[1][0]*p_mpv->v.ekf.Jh_trans[0][1]+p_mpv->v.ekf.Pk_est[1][1]*p_mpv->v.ekf.Jh_trans[1][1]+p_mpv->v.ekf.Pk_est[1][2]*p_mpv->v.ekf.Jh_trans[2][1]+p_mpv->v.ekf.Pk_est[1][3]*p_mpv->v.ekf.Jh_trans[3][1];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[2][0] = p_mpv->v.ekf.Pk_est[2][0]*p_mpv->v.ekf.Jh_trans[0][0]+p_mpv->v.ekf.Pk_est[2][1]*p_mpv->v.ekf.Jh_trans[1][0]+p_mpv->v.ekf.Pk_est[2][2]*p_mpv->v.ekf.Jh_trans[2][0]+p_mpv->v.ekf.Pk_est[2][3]*p_mpv->v.ekf.Jh_trans[3][0];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[2][1] = p_mpv->v.ekf.Pk_est[2][0]*p_mpv->v.ekf.Jh_trans[0][1]+p_mpv->v.ekf.Pk_est[2][1]*p_mpv->v.ekf.Jh_trans[1][1]+p_mpv->v.ekf.Pk_est[2][2]*p_mpv->v.ekf.Jh_trans[2][1]+p_mpv->v.ekf.Pk_est[2][3]*p_mpv->v.ekf.Jh_trans[3][1];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[3][0] = p_mpv->v.ekf.Pk_est[3][0]*p_mpv->v.ekf.Jh_trans[0][0]+p_mpv->v.ekf.Pk_est[3][1]*p_mpv->v.ekf.Jh_trans[1][0]+p_mpv->v.ekf.Pk_est[3][2]*p_mpv->v.ekf.Jh_trans[2][0]+p_mpv->v.ekf.Pk_est[3][3]*p_mpv->v.ekf.Jh_trans[3][0];
    p_mpv->v.ekf.multi_Pkest_Jhtrans[3][1] = p_mpv->v.ekf.Pk_est[3][0]*p_mpv->v.ekf.Jh_trans[0][1]+p_mpv->v.ekf.Pk_est[3][1]*p_mpv->v.ekf.Jh_trans[1][1]+p_mpv->v.ekf.Pk_est[3][2]*p_mpv->v.ekf.Jh_trans[2][1]+p_mpv->v.ekf.Pk_est[3][3]*p_mpv->v.ekf.Jh_trans[3][1];

    p_mpv->v.ekf.multi_Jh_Pkest[0][0] = p_mpv->v.ekf.Jh[0][0]*p_mpv->v.ekf.Pk_est[0][0]+p_mpv->v.ekf.Jh[0][1]*p_mpv->v.ekf.Pk_est[1][0]+p_mpv->v.ekf.Jh[0][2]*p_mpv->v.ekf.Pk_est[2][0]+p_mpv->v.ekf.Jh[0][3]*p_mpv->v.ekf.Pk_est[3][0];
    p_mpv->v.ekf.multi_Jh_Pkest[0][1] = p_mpv->v.ekf.Jh[0][0]*p_mpv->v.ekf.Pk_est[0][1]+p_mpv->v.ekf.Jh[0][1]*p_mpv->v.ekf.Pk_est[1][1]+p_mpv->v.ekf.Jh[0][2]*p_mpv->v.ekf.Pk_est[2][1]+p_mpv->v.ekf.Jh[0][3]*p_mpv->v.ekf.Pk_est[3][1];
    p_mpv->v.ekf.multi_Jh_Pkest[0][2] = p_mpv->v.ekf.Jh[0][0]*p_mpv->v.ekf.Pk_est[0][2]+p_mpv->v.ekf.Jh[0][1]*p_mpv->v.ekf.Pk_est[1][2]+p_mpv->v.ekf.Jh[0][2]*p_mpv->v.ekf.Pk_est[2][2]+p_mpv->v.ekf.Jh[0][3]*p_mpv->v.ekf.Pk_est[3][2];
    p_mpv->v.ekf.multi_Jh_Pkest[0][3] = p_mpv->v.ekf.Jh[0][0]*p_mpv->v.ekf.Pk_est[0][3]+p_mpv->v.ekf.Jh[0][1]*p_mpv->v.ekf.Pk_est[1][3]+p_mpv->v.ekf.Jh[0][2]*p_mpv->v.ekf.Pk_est[2][3]+p_mpv->v.ekf.Jh[0][3]*p_mpv->v.ekf.Pk_est[3][3];
    p_mpv->v.ekf.multi_Jh_Pkest[1][0] = p_mpv->v.ekf.Jh[1][0]*p_mpv->v.ekf.Pk_est[0][0]+p_mpv->v.ekf.Jh[1][1]*p_mpv->v.ekf.Pk_est[1][0]+p_mpv->v.ekf.Jh[1][2]*p_mpv->v.ekf.Pk_est[2][0]+p_mpv->v.ekf.Jh[1][3]*p_mpv->v.ekf.Pk_est[3][0];
    p_mpv->v.ekf.multi_Jh_Pkest[1][1] = p_mpv->v.ekf.Jh[1][0]*p_mpv->v.ekf.Pk_est[0][1]+p_mpv->v.ekf.Jh[1][1]*p_mpv->v.ekf.Pk_est[1][1]+p_mpv->v.ekf.Jh[1][2]*p_mpv->v.ekf.Pk_est[2][1]+p_mpv->v.ekf.Jh[1][3]*p_mpv->v.ekf.Pk_est[3][1];
    p_mpv->v.ekf.multi_Jh_Pkest[1][2] = p_mpv->v.ekf.Jh[1][0]*p_mpv->v.ekf.Pk_est[0][2]+p_mpv->v.ekf.Jh[1][1]*p_mpv->v.ekf.Pk_est[1][2]+p_mpv->v.ekf.Jh[1][2]*p_mpv->v.ekf.Pk_est[2][2]+p_mpv->v.ekf.Jh[1][3]*p_mpv->v.ekf.Pk_est[3][2];
    p_mpv->v.ekf.multi_Jh_Pkest[1][3] = p_mpv->v.ekf.Jh[1][0]*p_mpv->v.ekf.Pk_est[0][3]+p_mpv->v.ekf.Jh[1][1]*p_mpv->v.ekf.Pk_est[1][3]+p_mpv->v.ekf.Jh[1][2]*p_mpv->v.ekf.Pk_est[2][3]+p_mpv->v.ekf.Jh[1][3]*p_mpv->v.ekf.Pk_est[3][3];

    p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[0][0] = p_mpv->v.ekf.multi_Jh_Pkest[0][0]*p_mpv->v.ekf.Jh_trans[0][0] + p_mpv->v.ekf.multi_Jh_Pkest[0][1]*p_mpv->v.ekf.Jh_trans[1][0] + p_mpv->v.ekf.multi_Jh_Pkest[0][2]*p_mpv->v.ekf.Jh_trans[2][0]+p_mpv->v.ekf.multi_Jh_Pkest[0][3]*p_mpv->v.ekf.Jh_trans[3][0];
    p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[0][1] = p_mpv->v.ekf.multi_Jh_Pkest[0][0]*p_mpv->v.ekf.Jh_trans[0][1] + p_mpv->v.ekf.multi_Jh_Pkest[0][1]*p_mpv->v.ekf.Jh_trans[1][1] + p_mpv->v.ekf.multi_Jh_Pkest[0][2]*p_mpv->v.ekf.Jh_trans[2][1]+p_mpv->v.ekf.multi_Jh_Pkest[0][3]*p_mpv->v.ekf.Jh_trans[3][1];
    p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[1][0] = p_mpv->v.ekf.multi_Jh_Pkest[1][0]*p_mpv->v.ekf.Jh_trans[0][0] + p_mpv->v.ekf.multi_Jh_Pkest[1][1]*p_mpv->v.ekf.Jh_trans[1][0] + p_mpv->v.ekf.multi_Jh_Pkest[1][2]*p_mpv->v.ekf.Jh_trans[2][0]+p_mpv->v.ekf.multi_Jh_Pkest[1][3]*p_mpv->v.ekf.Jh_trans[3][0];
    p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[1][1] = p_mpv->v.ekf.multi_Jh_Pkest[1][0]*p_mpv->v.ekf.Jh_trans[0][1] + p_mpv->v.ekf.multi_Jh_Pkest[1][1]*p_mpv->v.ekf.Jh_trans[1][1] + p_mpv->v.ekf.multi_Jh_Pkest[1][2]*p_mpv->v.ekf.Jh_trans[2][1]+p_mpv->v.ekf.multi_Jh_Pkest[1][3]*p_mpv->v.ekf.Jh_trans[3][1];

    p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[0][0] = p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[0][0]+p_mpv->v.ekf.Rk_real[0][0];
    p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[0][1] = p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[0][1]+p_mpv->v.ekf.Rk_real[0][1];
    p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[1][0] = p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[1][0]+p_mpv->v.ekf.Rk_real[1][0];
    p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[1][1] = p_mpv->v.ekf.multi_Jh_Pkest_Jhtrans[1][1]+p_mpv->v.ekf.Rk_real[1][1];

    p_mpv->v.ekf.invDet = 1.0/(p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[0][0]*p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[1][1]-p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[0][1]*p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[1][0]);
    p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][0] = p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[1][1]*p_mpv->v.ekf.invDet;
    p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][1] = -(p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[0][1]*p_mpv->v.ekf.invDet);
    p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][0] = -(p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[1][0]*p_mpv->v.ekf.invDet);
    p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][1] = p_mpv->v.ekf.sum_Jh_Pkest_Jhtrans_Rk[0][0]*p_mpv->v.ekf.invDet;

    p_mpv->v.ekf.Kk[0][0] = p_mpv->v.ekf.multi_Pkest_Jhtrans[0][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][0] + p_mpv->v.ekf.multi_Pkest_Jhtrans[0][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][0];
    p_mpv->v.ekf.Kk[0][1] = p_mpv->v.ekf.multi_Pkest_Jhtrans[0][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][1] + p_mpv->v.ekf.multi_Pkest_Jhtrans[0][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][1];
    p_mpv->v.ekf.Kk[1][0] = p_mpv->v.ekf.multi_Pkest_Jhtrans[1][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][0] + p_mpv->v.ekf.multi_Pkest_Jhtrans[1][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][0];
    p_mpv->v.ekf.Kk[1][1] = p_mpv->v.ekf.multi_Pkest_Jhtrans[1][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][1] + p_mpv->v.ekf.multi_Pkest_Jhtrans[1][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][1];
    p_mpv->v.ekf.Kk[2][0] = p_mpv->v.ekf.multi_Pkest_Jhtrans[2][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][0] + p_mpv->v.ekf.multi_Pkest_Jhtrans[2][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][0];
    p_mpv->v.ekf.Kk[2][1] = p_mpv->v.ekf.multi_Pkest_Jhtrans[2][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][1] + p_mpv->v.ekf.multi_Pkest_Jhtrans[2][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][1];
    p_mpv->v.ekf.Kk[3][0] = p_mpv->v.ekf.multi_Pkest_Jhtrans[3][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][0] + p_mpv->v.ekf.multi_Pkest_Jhtrans[3][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][0];
    p_mpv->v.ekf.Kk[3][1] = p_mpv->v.ekf.multi_Pkest_Jhtrans[3][0]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[0][1] + p_mpv->v.ekf.multi_Pkest_Jhtrans[3][1]*p_mpv->v.ekf.inv_sum_Jh_Pkest_Jhtrans_Rk[1][1];

    p_mpv->v.ekf.abstract_zk_hxk[0][0] = p_mpv->v.ekf.zk[0][0] - p_mpv->v.ekf.hxk[0][0];
    p_mpv->v.ekf.abstract_zk_hxk[1][0] = p_mpv->v.ekf.zk[1][0] - p_mpv->v.ekf.hxk[1][0];

    p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[0][0] = p_mpv->v.ekf.Kk[0][0]*p_mpv->v.ekf.abstract_zk_hxk[0][0] + p_mpv->v.ekf.Kk[0][1]*p_mpv->v.ekf.abstract_zk_hxk[1][0];
    p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[1][0] = p_mpv->v.ekf.Kk[1][0]*p_mpv->v.ekf.abstract_zk_hxk[0][0] + p_mpv->v.ekf.Kk[1][1]*p_mpv->v.ekf.abstract_zk_hxk[1][0];
    p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[2][0] = p_mpv->v.ekf.Kk[2][0]*p_mpv->v.ekf.abstract_zk_hxk[0][0] + p_mpv->v.ekf.Kk[2][1]*p_mpv->v.ekf.abstract_zk_hxk[1][0];
    p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[3][0] = p_mpv->v.ekf.Kk[3][0]*p_mpv->v.ekf.abstract_zk_hxk[0][0] + p_mpv->v.ekf.Kk[3][1]*p_mpv->v.ekf.abstract_zk_hxk[1][0];

    p_mpv->v.ekf.xk_inno[0][0] = p_mpv->v.ekf.xk_est[0][0] + p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[0][0];
    p_mpv->v.ekf.xk_inno[1][0] = p_mpv->v.ekf.xk_est[1][0] + p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[1][0];
    p_mpv->v.ekf.xk_inno[2][0] = p_mpv->v.ekf.xk_est[2][0] + p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[2][0];
    p_mpv->v.ekf.xk_inno[3][0] = p_mpv->v.ekf.xk_est[3][0] + p_mpv->v.ekf.multi_Kk_abstract_zk_hxk[3][0];

    p_mpv->v.ekf.multi_Kk_Jh[0][0] = p_mpv->v.ekf.Kk[0][0]*p_mpv->v.ekf.Jh[0][0] + p_mpv->v.ekf.Kk[0][1]*p_mpv->v.ekf.Jh[1][0];
    p_mpv->v.ekf.multi_Kk_Jh[0][1] = p_mpv->v.ekf.Kk[0][0]*p_mpv->v.ekf.Jh[0][1] + p_mpv->v.ekf.Kk[0][1]*p_mpv->v.ekf.Jh[1][1];
    p_mpv->v.ekf.multi_Kk_Jh[0][2] = p_mpv->v.ekf.Kk[0][0]*p_mpv->v.ekf.Jh[0][2] + p_mpv->v.ekf.Kk[0][1]*p_mpv->v.ekf.Jh[1][2];
    p_mpv->v.ekf.multi_Kk_Jh[0][3] = p_mpv->v.ekf.Kk[0][0]*p_mpv->v.ekf.Jh[0][3] + p_mpv->v.ekf.Kk[0][1]*p_mpv->v.ekf.Jh[1][3];
    p_mpv->v.ekf.multi_Kk_Jh[1][0] = p_mpv->v.ekf.Kk[1][0]*p_mpv->v.ekf.Jh[0][0] + p_mpv->v.ekf.Kk[1][1]*p_mpv->v.ekf.Jh[1][0];
    p_mpv->v.ekf.multi_Kk_Jh[1][1] = p_mpv->v.ekf.Kk[1][0]*p_mpv->v.ekf.Jh[0][1] + p_mpv->v.ekf.Kk[1][1]*p_mpv->v.ekf.Jh[1][1];
    p_mpv->v.ekf.multi_Kk_Jh[1][2] = p_mpv->v.ekf.Kk[1][0]*p_mpv->v.ekf.Jh[0][2] + p_mpv->v.ekf.Kk[1][1]*p_mpv->v.ekf.Jh[1][2];
    p_mpv->v.ekf.multi_Kk_Jh[1][3] = p_mpv->v.ekf.Kk[1][0]*p_mpv->v.ekf.Jh[0][3] + p_mpv->v.ekf.Kk[1][1]*p_mpv->v.ekf.Jh[1][3];
    p_mpv->v.ekf.multi_Kk_Jh[2][0] = p_mpv->v.ekf.Kk[2][0]*p_mpv->v.ekf.Jh[0][0] + p_mpv->v.ekf.Kk[2][1]*p_mpv->v.ekf.Jh[1][0];
    p_mpv->v.ekf.multi_Kk_Jh[2][1] = p_mpv->v.ekf.Kk[2][0]*p_mpv->v.ekf.Jh[0][1] + p_mpv->v.ekf.Kk[2][1]*p_mpv->v.ekf.Jh[1][1];
    p_mpv->v.ekf.multi_Kk_Jh[2][2] = p_mpv->v.ekf.Kk[2][0]*p_mpv->v.ekf.Jh[0][2] + p_mpv->v.ekf.Kk[2][1]*p_mpv->v.ekf.Jh[1][2];
    p_mpv->v.ekf.multi_Kk_Jh[2][3] = p_mpv->v.ekf.Kk[2][0]*p_mpv->v.ekf.Jh[0][3] + p_mpv->v.ekf.Kk[2][1]*p_mpv->v.ekf.Jh[1][3];
    p_mpv->v.ekf.multi_Kk_Jh[3][0] = p_mpv->v.ekf.Kk[3][0]*p_mpv->v.ekf.Jh[0][0] + p_mpv->v.ekf.Kk[3][1]*p_mpv->v.ekf.Jh[1][0];
    p_mpv->v.ekf.multi_Kk_Jh[3][1] = p_mpv->v.ekf.Kk[3][0]*p_mpv->v.ekf.Jh[0][1] + p_mpv->v.ekf.Kk[3][1]*p_mpv->v.ekf.Jh[1][1];
    p_mpv->v.ekf.multi_Kk_Jh[3][2] = p_mpv->v.ekf.Kk[3][0]*p_mpv->v.ekf.Jh[0][2] + p_mpv->v.ekf.Kk[3][1]*p_mpv->v.ekf.Jh[1][2];
    p_mpv->v.ekf.multi_Kk_Jh[3][3] = p_mpv->v.ekf.Kk[3][0]*p_mpv->v.ekf.Jh[0][3] + p_mpv->v.ekf.Kk[3][1]*p_mpv->v.ekf.Jh[1][3];

    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][0] = p_mpv->v.ekf.I_EKF[0][0]-p_mpv->v.ekf.multi_Kk_Jh[0][0];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][1] = p_mpv->v.ekf.I_EKF[0][1]-p_mpv->v.ekf.multi_Kk_Jh[0][1];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][2] = p_mpv->v.ekf.I_EKF[0][2]-p_mpv->v.ekf.multi_Kk_Jh[0][2];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][3] = p_mpv->v.ekf.I_EKF[0][3]-p_mpv->v.ekf.multi_Kk_Jh[0][3];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][0] = p_mpv->v.ekf.I_EKF[1][0]-p_mpv->v.ekf.multi_Kk_Jh[1][0];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][1] = p_mpv->v.ekf.I_EKF[1][1]-p_mpv->v.ekf.multi_Kk_Jh[1][1];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][2] = p_mpv->v.ekf.I_EKF[1][2]-p_mpv->v.ekf.multi_Kk_Jh[1][2];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][3] = p_mpv->v.ekf.I_EKF[1][3]-p_mpv->v.ekf.multi_Kk_Jh[1][3];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][0] = p_mpv->v.ekf.I_EKF[2][0]-p_mpv->v.ekf.multi_Kk_Jh[2][0];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][1] = p_mpv->v.ekf.I_EKF[2][1]-p_mpv->v.ekf.multi_Kk_Jh[2][1];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][2] = p_mpv->v.ekf.I_EKF[2][2]-p_mpv->v.ekf.multi_Kk_Jh[2][2];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][3] = p_mpv->v.ekf.I_EKF[2][3]-p_mpv->v.ekf.multi_Kk_Jh[2][3];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][0] = p_mpv->v.ekf.I_EKF[3][0]-p_mpv->v.ekf.multi_Kk_Jh[3][0];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][1] = p_mpv->v.ekf.I_EKF[3][1]-p_mpv->v.ekf.multi_Kk_Jh[3][1];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][2] = p_mpv->v.ekf.I_EKF[3][2]-p_mpv->v.ekf.multi_Kk_Jh[3][2];
    p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][3] = p_mpv->v.ekf.I_EKF[3][3]-p_mpv->v.ekf.multi_Kk_Jh[3][3];

    p_mpv->v.ekf.Pk_inno[0][0] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][0]*p_mpv->v.ekf.Pk_est[0][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][1]*p_mpv->v.ekf.Pk_est[1][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][3]*p_mpv->v.ekf.Pk_est[3][0];
    p_mpv->v.ekf.Pk_inno[0][1] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][0]*p_mpv->v.ekf.Pk_est[0][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][1]*p_mpv->v.ekf.Pk_est[1][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][3]*p_mpv->v.ekf.Pk_est[3][1];
    p_mpv->v.ekf.Pk_inno[0][2] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][0]*p_mpv->v.ekf.Pk_est[0][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][1]*p_mpv->v.ekf.Pk_est[1][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][3]*p_mpv->v.ekf.Pk_est[3][2];
    p_mpv->v.ekf.Pk_inno[0][3] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][0]*p_mpv->v.ekf.Pk_est[0][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][1]*p_mpv->v.ekf.Pk_est[1][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[0][3]*p_mpv->v.ekf.Pk_est[3][3];
    p_mpv->v.ekf.Pk_inno[1][0] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][0]*p_mpv->v.ekf.Pk_est[0][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][1]*p_mpv->v.ekf.Pk_est[1][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][3]*p_mpv->v.ekf.Pk_est[3][0];
    p_mpv->v.ekf.Pk_inno[1][1] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][0]*p_mpv->v.ekf.Pk_est[0][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][1]*p_mpv->v.ekf.Pk_est[1][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][3]*p_mpv->v.ekf.Pk_est[3][1];
    p_mpv->v.ekf.Pk_inno[1][2] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][0]*p_mpv->v.ekf.Pk_est[0][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][1]*p_mpv->v.ekf.Pk_est[1][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][3]*p_mpv->v.ekf.Pk_est[3][2];
    p_mpv->v.ekf.Pk_inno[1][3] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][0]*p_mpv->v.ekf.Pk_est[0][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][1]*p_mpv->v.ekf.Pk_est[1][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[1][3]*p_mpv->v.ekf.Pk_est[3][3];
    p_mpv->v.ekf.Pk_inno[2][0] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][0]*p_mpv->v.ekf.Pk_est[0][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][1]*p_mpv->v.ekf.Pk_est[1][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][3]*p_mpv->v.ekf.Pk_est[3][0];
    p_mpv->v.ekf.Pk_inno[2][1] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][0]*p_mpv->v.ekf.Pk_est[0][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][1]*p_mpv->v.ekf.Pk_est[1][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][3]*p_mpv->v.ekf.Pk_est[3][1];
    p_mpv->v.ekf.Pk_inno[2][2] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][0]*p_mpv->v.ekf.Pk_est[0][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][1]*p_mpv->v.ekf.Pk_est[1][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][3]*p_mpv->v.ekf.Pk_est[3][2];
    p_mpv->v.ekf.Pk_inno[2][3] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][0]*p_mpv->v.ekf.Pk_est[0][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][1]*p_mpv->v.ekf.Pk_est[1][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[2][3]*p_mpv->v.ekf.Pk_est[3][3];
    p_mpv->v.ekf.Pk_inno[3][0] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][0]*p_mpv->v.ekf.Pk_est[0][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][1]*p_mpv->v.ekf.Pk_est[1][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][2]*p_mpv->v.ekf.Pk_inno_old[2][0]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][3]*p_mpv->v.ekf.Pk_est[3][0];
    p_mpv->v.ekf.Pk_inno[3][1] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][0]*p_mpv->v.ekf.Pk_est[0][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][1]*p_mpv->v.ekf.Pk_est[1][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][2]*p_mpv->v.ekf.Pk_inno_old[2][1]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][3]*p_mpv->v.ekf.Pk_est[3][1];
    p_mpv->v.ekf.Pk_inno[3][2] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][0]*p_mpv->v.ekf.Pk_est[0][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][1]*p_mpv->v.ekf.Pk_est[1][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][2]*p_mpv->v.ekf.Pk_inno_old[2][2]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][3]*p_mpv->v.ekf.Pk_est[3][2];
    p_mpv->v.ekf.Pk_inno[3][3] = p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][0]*p_mpv->v.ekf.Pk_est[0][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][1]*p_mpv->v.ekf.Pk_est[1][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][2]*p_mpv->v.ekf.Pk_inno_old[2][3]+p_mpv->v.ekf.abstract_I_multi_Kk_Jh[3][3]*p_mpv->v.ekf.Pk_est[3][3];

    if(p_mpv->v.ekf.xk_inno[3][0]>=(2*MC_PI))
    {
	p_mpv->v.ekf.xk_inno[3][0]-=2*MC_PI;
    }
    /*
    else if(p_mpv->v.ekf.xk_inno[3][0]<0)
    {
	p_mpv->v.ekf.xk_inno[3][0]+=2*MC_PI;
    }
    */
    else
    {
    }

    if(p_mpv->v.state == RAMP)
    {
	p_mpv->v.ekf.Qk_real[0][0] = p_mpv->p.ekf.Qk_cls[0][0];
	p_mpv->v.ekf.Qk_real[1][1] = p_mpv->p.ekf.Qk_cls[0][0];
	p_mpv->v.ekf.theta_EKF_Shift_real =p_mpv->p.ekf.theta_EKF_Shift;

    }
    else
    {
    }

    p_mpv->v.ekf.xk_inno[3][0]+=p_mpv->v.ekf.theta_EKF_Shift_real;

    if(p_mpv->v.ekf.xk_inno[3][0]>=(2*MC_PI))
    {
	p_mpv->v.ekf.xk_inno[3][0]-=2*MC_PI;
    }
    /*
    else if(p_mpv->v.ekf.xk_inno[3][0]<0)
    {
	p_mpv->v.ekf.xk_inno[3][0]+=2*MC_PI;
    }
    */
    else
    {
    }

    p_mpv->v.ekf.xk_inno_filt = LPF_FirstOrder(p_mpv->v.ekf.prev_Theta, p_mpv->v.ekf.xk_inno[3][0], p_mpv->v.periodFL,p_mpv->p.ekf.fcutTheta);



    if(p_mpv->v.ekf.xk_inno_old_filt > (p_mpv->v.ekf.xk_inno_filt+(MC_PI)))
    {
	p_mpv->v.ekf.xk_inno_old_filt -= 2*MC_PI;
    }

    ++p_mpv->v.ekf.speed_cnt_EKF;

    p_mpv->v.ekf.theta_delta_EKF = p_mpv->v.ekf.xk_inno_filt - p_mpv->v.ekf.xk_inno_old_filt;
    p_mpv->v.ekf.theta_sum_EKF += p_mpv->v.ekf.theta_delta_EKF;

    if(p_mpv->v.ekf.speed_cnt_EKF >= p_mpv->p.ekf.sampling_speed_EKF)
    {
	p_mpv->v.ekf.delta_theta_ave_EKF = p_mpv->v.ekf.theta_sum_EKF/p_mpv->v.ekf.speed_cnt_EKF;
        p_mpv->v.ekf.speed_EKF=(p_mpv->v.ekf.delta_theta_ave_EKF*(p_mpv->p.freqFL))/6/3*p_mpv->p.ekf.speed_corr_EKF;
	p_mpv->v.ekf.theta_sum_EKF -= p_mpv->v.ekf.delta_theta_ave_EKF;
        p_mpv->v.ekf.speed_cnt_EKF = p_mpv->p.ekf.sampling_speed_EKF - 1;
    }

    p_mpv->v.ekf.speed_EKF_Filt = LPF_FirstOrder(p_mpv->v.ekf.prev_speed_EKF, p_mpv->v.ekf.speed_EKF, p_mpv->v.periodFL, p_mpv->p.ekf.fcutSpeed);

    p_mpv->v.ekf.xk_inno_old[0][0] = p_mpv->v.ekf.xk_inno[0][0];
    p_mpv->v.ekf.xk_inno_old[1][0] = p_mpv->v.ekf.xk_inno[1][0];
    p_mpv->v.ekf.xk_inno_old[2][0] = p_mpv->v.ekf.xk_inno[2][0];
    p_mpv->v.ekf.xk_inno_old[3][0] = p_mpv->v.ekf.xk_inno[3][0];

    p_mpv->v.ekf.xk_inno_old_filt = p_mpv->v.ekf.xk_inno_filt;

    p_mpv->v.ekf.Pk_inno_old[0][0] = p_mpv->v.ekf.Pk_inno[0][0];
    p_mpv->v.ekf.Pk_inno_old[0][1] = p_mpv->v.ekf.Pk_inno[0][1];
    p_mpv->v.ekf.Pk_inno_old[0][2] = p_mpv->v.ekf.Pk_inno[0][2];
    p_mpv->v.ekf.Pk_inno_old[0][3] = p_mpv->v.ekf.Pk_inno[0][3];
    p_mpv->v.ekf.Pk_inno_old[1][0] = p_mpv->v.ekf.Pk_inno[1][0];
    p_mpv->v.ekf.Pk_inno_old[1][1] = p_mpv->v.ekf.Pk_inno[1][1];
    p_mpv->v.ekf.Pk_inno_old[1][2] = p_mpv->v.ekf.Pk_inno[1][2];
    p_mpv->v.ekf.Pk_inno_old[1][3] = p_mpv->v.ekf.Pk_inno[1][3];
    p_mpv->v.ekf.Pk_inno_old[2][0] = p_mpv->v.ekf.Pk_inno[2][0];
    p_mpv->v.ekf.Pk_inno_old[2][1] = p_mpv->v.ekf.Pk_inno[2][1];
    p_mpv->v.ekf.Pk_inno_old[2][2] = p_mpv->v.ekf.Pk_inno[2][2];
    p_mpv->v.ekf.Pk_inno_old[2][3] = p_mpv->v.ekf.Pk_inno[2][3];
    p_mpv->v.ekf.Pk_inno_old[3][0] = p_mpv->v.ekf.Pk_inno[3][0];
    p_mpv->v.ekf.Pk_inno_old[3][1] = p_mpv->v.ekf.Pk_inno[3][1];
    p_mpv->v.ekf.Pk_inno_old[3][2] = p_mpv->v.ekf.Pk_inno[3][2];
    p_mpv->v.ekf.Pk_inno_old[3][3] = p_mpv->v.ekf.Pk_inno[3][3];

    p_mpv->v.ekf.prev_Theta = p_mpv->v.ekf.xk_inno_filt;
    p_mpv->v.ekf.prev_speed_EKF = p_mpv->v.ekf.speed_EKF_Filt;

    p_mpv->v.ekf.theta_EKF_deg=p_mpv->v.ekf.xk_inno_filt*MC_RAD2DEG;
}

static void mcRobustBemfObserverHandlerSL(mcCpv_t *p_mpv)
{
    if(mathAbsoluteFlt(p_mpv->v.spref) > p_mpv->p.minsp_o2c)
    {
        p_mpv->v.ato.k1_unmerged = p_mpv->p.ato.k1min + p_mpv->v.ato.k1_coeff * 
                    (mathAbsoluteFlt(p_mpv->v.spref) - p_mpv->p.minsp_o2c);
    }
    else
    {
        p_mpv->v.ato.k1_unmerged = p_mpv->p.ato.k1min;
        p_mpv->v.ato.k1_merge_coeff = 1.0f;
    }
    p_mpv->v.ato.k1 = mathMergeFlt(p_mpv->v.ato.k1_unmerged, p_mpv->v.ato.k1, p_mpv->p.ato.k1_merge_step, &p_mpv->v.ato.k1_merge_coeff);
}
