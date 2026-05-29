/*
 * pmsm_rk4.c Integracao RK4 do modelo PMSM em dq-axis.
 * Autor: EDUARDO HENRIQUE COUTO
 * Requisitos: C99, <math.h>
 * Date: Dez/2012
 * Estados: x = [id, iq, omega_m, theta_e]
 * Entradas: u = [vd, vq, T_L]
 * Sa�das: y = [Te, we]
 *
 * Notas:
 *  - Pneum.  unidades SI (H, Ohm, Wb, N.m, rad/s).
 *  - theta_e envolvido em [0, 2*pi).
 *  - Suporta float (default) ou double (defina PMSM_USE_DOUBLE=1).
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include"pmsm_rk4.h"

/* ========================= Configuracao do modulo ========================= */
#define MATH__SATURATE_DIRECT(lim_inf, value, lim_sup)                                          \
 (((value) >= (lim_sup)) ? (lim_sup) : ( ((value) <= (lim_inf)) ? (lim_inf) : (value) ))            \

#define TWO_PI         (2.0f * PMSM_PI)
//#define SW_FREQ        8000.0f

/* Initalize prototypes */
static PMSMParams Motor0;
static PMSMState States;
static PMSMInputs Inputs;
static PMSMOutputs Outputs;

static real_t sin_theta;
static real_t cos_theta;

static void CurrentReconstruction(PMSMOutputs *out);

/* ========================= Initializacao do Motor =========================== */
void pmsm_initialize(PMSMParams params)
{
    //Load Motor Parameters
    Motor0 = params;

    //Initialize States & Outputs
    States.id = 0.0f;
    States.iq = 0.0f;
    States.omega_m = 0.0f;
    States.theta_e = 0.0f;
    Outputs.Te = 0.0f;

    // Set Inputs to Default Values
    Inputs.T_L = 0.0f;
    Inputs.vd  = 0.0f;
    Inputs.vq  = 0.0f;
}


/* ===================== Dinamica continua (derivadas) ======================= */
inline static void pmsm_state_eq(
    PMSMParams  *params,
    PMSMState   *sts,
    PMSMInputs  *in,
    PMSMState   *dx
				                )
{

    real_t id = sts->id;
    real_t iq = sts->iq;
    (void)sts->theta_e; // nao usado nas equacoes dq ideais
    real_t wm;
    real_t we;

    wm = sts->omega_m;
    we = (real_t)(params->p) * wm;

    // Eletrico
    dx->id = (in->vd - params->Rs * id + we * params->Lq * iq) / params->Ld;
    dx->iq = (in->vq - params->Rs * iq - we * (params->Ld * id + params->Phi)) / params->Lq;

    // Torque (inclui termo de relutancia para IPMSM)
    real_t Te = (real_t)1.5 * (real_t)(params->p) * (params->Phi * iq + (params->Ld - params->Lq) * id * iq);

    // Mecanico
    dx->omega_m = (Te - in->T_L - params->B * wm) * params->J_Inv;
    dx->theta_e = we;
}

/* ========================= Passo RK4 com subpassos ========================= */
void pmsm_rk4_step(
	    PMSMState         *states,       // [in/out] estado atual e atualizado
	    PMSMInputs        *inputs,       // [in]  entradas (mantidas constantes durante Ts)
	    PMSMParams        *params,       // [in]  parametros
	    PMSMOutputs       *outputs,      // [out] saidas no final do passo (pode ser NULL)
	    real_t             Ts            // [in]  periodo de integracao [s]
)
{
    real_t ts_6;
    PMSMState k1, k2, k3, k4, xt;
    PMSMState sk = *states;

    // k1 = f(xk)
    pmsm_state_eq(params, &sk, inputs, &k1);

    // k2 = f(xk + 0.5*h*k1)
    xt.id       = sk.id       + (real_t)0.5 * Ts * k1.id;
    xt.iq       = sk.iq       + (real_t)0.5 * Ts * k1.iq;
    xt.omega_m  = sk.omega_m  + (real_t)0.5 * Ts * k1.omega_m;
    xt.theta_e  = sk.theta_e  + (real_t)0.5 * Ts * k1.theta_e;
    pmsm_state_eq(params, &xt, inputs, &k2);

    // k3 = f(xk + 0.5*h*k2)
    xt.id       = sk.id       + (real_t)0.5 * Ts * k2.id;
    xt.iq       = sk.iq       + (real_t)0.5 * Ts * k2.iq;
    xt.omega_m  = sk.omega_m  + (real_t)0.5 * Ts * k2.omega_m;
    xt.theta_e  = sk.theta_e  + (real_t)0.5 * Ts * k2.theta_e;
    pmsm_state_eq(params, &xt, inputs, &k3);

    // k4 = f(xk + h*k3)
    xt.id       = sk.id       + Ts * k3.id;
    xt.iq       = sk.iq       + Ts * k3.iq;
    xt.omega_m  = sk.omega_m  + Ts * k3.omega_m;
    xt.theta_e  = sk.theta_e  + Ts * k3.theta_e;
    pmsm_state_eq(params, &xt, inputs, &k4);

    // x_{k+1} = xk + h/6*(k1 + 2*k2 + 2*k3 + k4)
    ts_6 = (Ts / (real_t)6.0);
    sk.id       += ts_6 * (k1.id      + (real_t)2.0 * k2.id      + (real_t)2.0 * k3.id      + k4.id);
    sk.iq       += ts_6 * (k1.iq      + (real_t)2.0 * k2.iq      + (real_t)2.0 * k3.iq      + k4.iq);
    sk.omega_m  += ts_6 * (k1.omega_m + (real_t)2.0 * k2.omega_m + (real_t)2.0 * k3.omega_m + k4.omega_m);
    sk.theta_e  += ts_6 * (k1.theta_e + (real_t)2.0 * k2.theta_e + (real_t)2.0 * k3.theta_e + k4.theta_e);

    // Envolve o angulo para manter numerica estavel
    if(sk.theta_e > TWO_PI) //SH
    {
    	sk.theta_e -= TWO_PI;
    }
    if(sk.theta_e < 0.0f) //SAH
    {
    	sk.theta_e += TWO_PI;
    }

    sin_theta = sinf(sk.theta_e);
    cos_theta = cosf(sk.theta_e);

    // Retorna estado final
    *states = sk;

    // Torque (inclui termo de relutancia para IPMSM)
    outputs->Te = 1.5f * (real_t)(params->p) * (params->Phi * sk.iq + (params->Ld - params->Lq) * sk.id * sk.iq);
    // Update Outputs
    outputs->Sts = sk; // (Opcional) disponibiliza estado atualizado para acesso direto
}

/* ============================ Exemplo de uso ============================ */
/*
#include <stdio.h>

int main(void) {
    pmsm_initialize();

    for (int k = 0; k < 20000; ++k) { // 2 s de simulacao
        // (Opcional) atualize u.vd, u.vq via controlador aqui
        pmsm_rk4_step(&States, &Inputs, &Motor1, &Outputs, Ts);

        if ((k % 1000) == 0) {
            printf("t=%.3f s | id=%.3f A, iq=%.3f A, wm=%.1f rad/s, Te=%.3f N.m\n",
                  (double)(k*Ts), (double)States.id, (double)States.iq, (double)States.omega_m, (double)Outputs.Te);
        }
    }
    return 0;
}*/

/* =================== External Callback ===================== */
void pmsm_step(float32 va, float32 vb, float32 tl, float32 ts)
{
    //Update input variables
    Inputs.vd = (va * cos_theta) + (vb * sin_theta); //vd = Park Transform from ab -> dq 
    Inputs.vq = ((-va) * sin_theta) + (vb * cos_theta); //vq = Park Transform from ab -> dq 
    Inputs.T_L = tl;

    //Reload motor model
    pmsm_rk4_step(&States, &Inputs, &Motor0, &Outputs, ts);

    //Generate 3ph Currents
    CurrentReconstruction(&Outputs);
}

/* ===================== Outputs API ======================= */
void pmsm_get_outputs(PMSMOutputs *outputs)
{
    *outputs = Outputs;
}


/* ================= Auxiliary Functions =================== */
void CurrentReconstruction(PMSMOutputs *out)
{
    real_t i_alpha;
    real_t i_beta;

    //Inverse Park Transform
    i_alpha = (out->Sts.id * cos_theta) - (out->Sts.iq * sin_theta);
    i_beta  = (out->Sts.id * sin_theta) + (out->Sts.iq * cos_theta);

    //Inverse Clarke Transform
    out->Iu = i_alpha;
    out->Iv = ((-i_alpha) + (sqrtf(3.0f) * i_beta)) / 2.0f;
    out->Iw = ((-i_alpha) - (sqrtf(3.0f) * i_beta)) / 2.0f;  
}