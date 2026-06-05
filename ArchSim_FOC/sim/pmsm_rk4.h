/*
 * pmsm_rk4.h � Integra��o RK4 do modelo PMSM em dq-axis.
 * Autor: EDUARDO HENRIQUE COUTO
 * Requisitos: C99, <math.h>
 * Date: Dez/2012
 * Estados: x = [id, iq, omega_m, theta_e]
 * Entradas: u = [vd, vq, T_L]
 * Sa�das: y = [Te, we]
 *
 * Notas:
 *  - Pneum. unidades SI (H, Ohm, Wb, N.m, rad/s).
 *  - theta_e � envolvido em [0, 2*pi].
 *  - Suporta float (default) ou double (defina PMSM_USE_DOUBLE=1).
 */

#ifndef PMSM_USE_DOUBLE
#define PMSM_USE_DOUBLE 0
#endif

#include "C_Extensions.h"

#if PMSM_USE_DOUBLE
    typedef double real_t;
    #define FABS        fabs
    #define FMOD        fmod
#else
    typedef float real_t;
    typedef int   int32_t;
    #define FABS        fabsf
    #define FMOD        fmodf
#endif

#define PMSM_INLINE inline

#ifndef PMSM_PI
#define PMSM_PI 3.14159265358979323846
#endif



/* ============================ Estruturas de dados =========================== */
typedef struct {
    int     p;     // Pares de polos
    real_t Rs;     // Resist�ncia de fase [Ohm]
    real_t Ld;     // Indut�ncia eixo d [H]
    real_t Lq;     // Indut�ncia eixo q [H]
    real_t Phi;    // Fluxo do �m� [Wb]
    real_t J_Inv;  // 1.0f / In�rcia [kg.m^2]
    real_t B;      // Atrito viscoso [N.m.s/rad]
} PMSMParams;

typedef struct {
    real_t id;       // Corrente eixo d [A]
    real_t iq;       // Corrente eixo q [A]
    real_t omega_m;  // Velocidade mec�nica [rad/s]
    real_t theta_e;  // Posi��o el�trica [rad]
} PMSMState;

typedef struct {
    real_t vd;   // Tens�o eixo d [V]
    real_t vq;   // Tens�o eixo q [V]
    real_t T_L;  // Torque de carga [N.m]
} PMSMInputs;

typedef struct {
    real_t Te;     // Torque eletromagn�tico [N.m]
    PMSMState Sts; // (Opcional) estado atualizado para acesso direto
    real_t Iu;            // 3-ph Phase Current U [A]
    real_t Iv;            // 3-ph Phase Current V [A]
    real_t Iw;            // 3-ph Phase Current W [A]
} PMSMOutputs;



void pmsm_initialize(PMSMParams params, float32 ts);
void pmsm_rk4_step(
    PMSMState         *states,       // [in/out] estado atual e atualizado
    PMSMInputs        *inputs,       // [in] entradas (mantidas constantes durante Ts)
    PMSMParams        *params,       // [in] par�metros
    PMSMOutputs       *outputs       // [out] sa�das no final do passo (pode ser NULL)
);
void pmsm_step(float32 va, float32 vb, float32 vc, float32 tl);
void pmsm_get_outputs(PMSMOutputs *outputs);