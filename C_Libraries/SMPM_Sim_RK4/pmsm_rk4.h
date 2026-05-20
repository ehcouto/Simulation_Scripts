/*
 * pmsm_rk4.h — Integração RK4 do modelo PMSM em dq-axis.
 * Autor: EDUARDO HENRIQUE COUTO
 * Requisitos: C99, <math.h>
 * Date: Dez/2012
 * Estados: x = [id, iq, omega_m, theta_e]
 * Entradas: u = [vd, vq, T_L]
 * Saídas: y = [Te, we]
 *
 * Notas:
 *  - Pneum. unidades SI (H, Ohm, Wb, N.m, rad/s).
 *  - theta_e é envolvido em [0, 2*pi].
 *  - Suporta float (default) ou double (defina PMSM_USE_DOUBLE=1).
 */

#ifndef PMSM_USE_DOUBLE
#define PMSM_USE_DOUBLE 0
#endif

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
    real_t Rs;     // Resistência de fase [Ohm]
    real_t Ld;     // Indutância eixo d [H]
    real_t Lq;     // Indutância eixo q [H]
    real_t Phi;    // Fluxo do ímã [Wb]
    real_t J_Inv;  // 1.0f / Inércia [kg.m^2]
    real_t B;      // Atrito viscoso [N.m.s/rad]
} PMSMParams;

typedef struct {
    real_t id;       // Corrente eixo d [A]
    real_t iq;       // Corrente eixo q [A]
    real_t omega_m;  // Velocidade mecânica [rad/s]
    real_t theta_e;  // Posição elétrica [rad]
} PMSMState;

typedef struct {
    real_t vd;   // Tensão eixo d [V]
    real_t vq;   // Tensão eixo q [V]
    real_t T_L;  // Torque de carga [N.m]
} PMSMInputs;

typedef struct {
    real_t Te;   // Torque eletromagnético [N.m]
    real_t we;   // Velocidade elétrica [rad/s]
} PMSMOutputs;



void pmsm_initialize(void);
void pmsm_rk4_step(
    PMSMState         *states,       // [in/out] estado atual e atualizado
    PMSMInputs        *inputs,       // [in] entradas (mantidas constantes durante Ts)
    PMSMParams        *params,       // [in] parâmetros
    PMSMOutputs       *outputs,      // [out] saídas no final do passo (pode ser NULL)
    real_t             Ts            // [in] período de integração [s]
);
