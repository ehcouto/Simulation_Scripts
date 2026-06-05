/*
 * Inverter Models
 * Autor: EDUARDO HENRIQUE COUTO
 * Requisitos: C99, <math.h>
 * Date: Jan/2013
 *
 * Notas:
 *  - Ideal Inverter
 *  - Inverter w/ deadtime insertion
 *  - Inverter w/ deadtime insertion + losses
 *  - Inverter w/ deadtime insertion + losses + delay 
 */
#include "inverter_model.h"
#include "Board_Sel.h"


    /* ********************************************* */
    /* ************ Module Variables *************** */ 
    /* ********************************************* */

#define SQRT3   1.73205080757f
#define MATHCALC__SATURATE_DIRECT(lim_inf, value, lim_sup)                                          \
 (((value) >= (lim_sup)) ? (lim_sup) : ( ((value) <= (lim_inf)) ? (lim_inf) : (value) ))            \

typedef struct {
    float duty_u_z1;
    float duty_v_z1;
    float duty_w_z1;
} InverterDelayState;


static float32 Inverter_DeadTime;
static float32 Inverter_Tpwm;
static InverterDelayState Inverter_Delay_State;

    /* ********************************************* */
    /* ************ Function Prototypes ************ */ 
    /* ********************************************* */

static float apply_deadtime(float D, float i_phase, float t_dt, float T_pwm);
static PhaseVoltages inverter_model_deadtime(InverterInput inv_in);






    /* ********************************************* */
    /* ************** Initialization *************** */ 
    /* ********************************************* */
 void inverter_model_init(float32 Tpwm)
 {
     Inverter_DeadTime = INVERTER_DEADTIME;
     Inverter_Tpwm = Tpwm;

     Inverter_Delay_State.duty_u_z1 = 0.0f;
     Inverter_Delay_State.duty_v_z1 = 0.0f;
     Inverter_Delay_State.duty_w_z1 = 0.0f;
 }



    /* ********************************************* */
    /* **************  Ideal Model   *************** */ 
    /* ********************************************* */
PhaseVoltages inverter_model_ideal(InverterInput inv_in)
{
    PhaseVoltages v;
    float temp; 
    
    temp = (inv_in.duty_u + inv_in.duty_v + inv_in.duty_w) / 3.0f;

    v.Va = inv_in.vdc * (inv_in.duty_u - temp);
    v.Vb = inv_in.vdc * (inv_in.duty_v - temp);
    v.Vc = inv_in.vdc * (inv_in.duty_w - temp);

    return v;
}



    /* ********************************************* */
    /* ************  DT + Losses Model   *********** */ 
    /* ********************************************* */
PhaseVoltages inverter_model_dt_losses(InverterInput inv_in)
{
    PhaseVoltages v = inverter_model_deadtime(inv_in);

    v.Va -= INVERTER_RES_EQ * inv_in.i_u;
    v.Vb -= INVERTER_RES_EQ * inv_in.i_v;
    v.Vc -= INVERTER_RES_EQ * inv_in.i_w;

    return v;
}




    /* ********************************************* */
    /* ********  DT + Losses Model + delay  ******** */ 
    /* ********************************************* */
PhaseVoltages inverter_model_delay(InverterInput inv_in)
{
    PhaseVoltages v;
    float duty_z1_u;
    float duty_z1_v;
    float duty_z1_w;

    //Save actual Duty Cycle context.
    duty_z1_u = inv_in.duty_u;
    duty_z1_v = inv_in.duty_v;
    duty_z1_w = inv_in.duty_w;

    //Load the current duty cycle with the previous ones (delay).
    inv_in.duty_u = Inverter_Delay_State.duty_u_z1;
    inv_in.duty_v = Inverter_Delay_State.duty_v_z1;
    inv_in.duty_w = Inverter_Delay_State.duty_w_z1;

    //Save next duty cycles.
    Inverter_Delay_State.duty_u_z1 = duty_z1_u;
    Inverter_Delay_State.duty_v_z1 = duty_z1_v;
    Inverter_Delay_State.duty_w_z1 = duty_z1_w;

    //Run inverter model (DT+Losses) w/ delayed duty cycles. 
    v = inverter_model_dt_losses(inv_in);

    return v;
}









    /* ********************************************* */
    /* ************  Auxiliary Functions  ********** */ 
    /* ********************************************* */

PhaseVoltages inverter_model_deadtime(InverterInput inv_in)
{
    float vdc, temp;
    PhaseVoltages v;

    //Apply Deadtime to the duty cycle signals
    float Da_eff = apply_deadtime(inv_in.duty_u, inv_in.i_u, Inverter_DeadTime, Inverter_Tpwm);
    float Db_eff = apply_deadtime(inv_in.duty_v, inv_in.i_v, Inverter_DeadTime, Inverter_Tpwm);
    float Dc_eff = apply_deadtime(inv_in.duty_w, inv_in.i_w, Inverter_DeadTime, Inverter_Tpwm);

    temp = (Da_eff + Db_eff + Dc_eff) / 3.0f;

    vdc = inv_in.vdc;

    //Calculate output voltage w/ new duty cycles
    v.Va = vdc * (Da_eff - temp);
    v.Vb = vdc * (Db_eff - temp);
    v.Vc = vdc * (Dc_eff - temp);

    return v;
}




float apply_deadtime(float D, float i_phase, float t_dt, float T_pwm)
{

    /* Deadtime Insertion Algorithm.
    Input:\(D_a, D_b, D_c, V_{dc}\), dead-time \(t_{dt}\), PWM period \(T_{pwm}\)
    
    Idea: dead-time reduces (or increases) the effective duty cycle depending on the phase current sign.
    A simple model per phase:
    
    \(\Delta D=\frac{t_{dt}}{T_{pwm}}\)
    
    If the phase current 
    
    \(i_a > 0\):
    
    the "effective" time of the upper transistor decreases → average voltage reduces:
    
    \(D_{a,eff}=D_{a}-\Delta D\)
    
    If \(i_a < 0\):
    
    the "effective" time of the lower transistor decreases → average voltage increases:
    
    \(D_{a,eff}=D_{a}+\Delta D\)
    
    Then apply the same scheme as the ideal model:
    
    \(D_{0}=\frac{D_{a,eff}+D_{b,eff}+D_{c,eff}}{3}\)\(V_{a}=V_{dc}\cdot (D_{a,eff}-D_{0})\)\(V_{b}=V_{dc}\cdot (D_{b,eff}-D_{0})\)\(V_{c}=V_{dc}\cdot (D_{c,eff}-D_{0})\) */

    float dD = t_dt / T_pwm;

    if (i_phase > 0.0f)
        return D - dD;   // reduz tensão média
    else if (i_phase < 0.0f)
        return D + dD;   // aumenta tensão média
    else
        return D;
}