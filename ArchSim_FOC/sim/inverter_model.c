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

#define  HIGH   1
#define  LOW    0

typedef struct {
    float32 duty_u_z1;
    float32 duty_v_z1;
    float32 duty_w_z1;
} InverterDelayState;



typedef struct {
    float32 dc_u; //Duty Cycle U
    float32 dc_v; //Duty Cycle V
    float32 dc_w; //Duty Cycle W
    float32 dt_u;
    float32 dt_v;
    float32 dt_w;
    uint8   Su_top;
    uint8   Sv_top;
    uint8   Sw_top;
    uint8   Su_bot;
    uint8   Sv_bot;
    uint8   Sw_bot;
} InverterPWMState;


static float32 Inverter_DeadTime;
static float32 Inverter_Tpwm;
static float32 Inverter_Tsim;
static float32 Inverter_DT_Tpwm;
static InverterDelayState Inverter_Delay_State;
static float32 Inverter_Carrier;
static float32 Inverter_Carrier_Step;
static sint8 Inverter_Carrier_Dir;
static InverterPWMState Inverter_PWM_State;

    /* ********************************************* */
    /* ************ Function Prototypes ************ */ 
    /* ********************************************* */

static float32 apply_deadtime(float32 D, float32 i_phase);
static PhaseVoltages inverter_model_deadtime(InverterInput inv_in);
static uint8 pwm_compare(float32 duty, float32 carrier);
static float32 phase_voltage(uint8 top, uint8 bot, float32 Vdc);
static void dt_insert_real(uint8 desired, uint8 *sx_top, float32 *dt_x);
static uint8 complementary_pwm(uint8 sx_top);





    /* ********************************************* */
    /* ************** Initialization *************** */ 
    /* ********************************************* */
 void inverter_model_init(float32 Tpwm, float32 t_sim)
 {
    Inverter_DeadTime = (float32)INVERTER_DEADTIME;
    Inverter_Tpwm = Tpwm;
    Inverter_Tsim = t_sim;
    Inverter_DT_Tpwm = (float32)INVERTER_DEADTIME / Tpwm;

    Inverter_Delay_State.duty_u_z1 = 0.0f;
    Inverter_Delay_State.duty_v_z1 = 0.0f;
    Inverter_Delay_State.duty_w_z1 = 0.0f;
    Inverter_Carrier = 0.0f;
    Inverter_Carrier_Step = (t_sim * 2.0f)/ Tpwm;
    Inverter_Carrier_Dir = DIRECTION_UP;

    // Init Real Inverter PWM (Bottom IGBT's ON)
    Inverter_PWM_State.dc_u = 0.0f;
    Inverter_PWM_State.dc_v = 0.0f;
    Inverter_PWM_State.dc_w = 0.0f;
    Inverter_PWM_State.dt_u = 0.0f;
    Inverter_PWM_State.dt_v = 0.0f;
    Inverter_PWM_State.dt_w = 0.0f;
    Inverter_PWM_State.Su_top = 0;
    Inverter_PWM_State.Sv_top = 0;
    Inverter_PWM_State.Sw_top = 0;
    Inverter_PWM_State.Su_bot = 1;
    Inverter_PWM_State.Sv_bot = 1;
    Inverter_PWM_State.Sw_bot = 1;
 }



    /* ********************************************* */
    /* **************  Ideal Model   *************** */ 
    /* ********************************************* */
PhaseVoltages inverter_model_ideal(InverterInput inv_in)
{
    PhaseVoltages v;
    float32 temp; 
    
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
    float32 duty_z1_u;
    float32 duty_z1_v;
    float32 duty_z1_w;

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
    /* **********  Inverter Real PWM  ************** */ 
    /* ********************************************* */
PhaseVoltages inverter_model_pwm(InverterInput inv_in)
{
    PhaseVoltages v;
    float32 va, vb, vc, vn;

    Inverter_Carrier += Inverter_Carrier_Step * Inverter_Carrier_Dir;

    if (Inverter_Carrier >= 1.0f) 
    {
        Inverter_Carrier = 1.0f;
        Inverter_Carrier_Dir = DIRECTION_DOWN;

        //Update the PWM only at the PWM crest
        Inverter_PWM_State.dc_u = MATHCALC__SATURATE_DIRECT(0.0f, inv_in.duty_u, 1.0f); //Limit Modulation at 100%
        Inverter_PWM_State.dc_v = MATHCALC__SATURATE_DIRECT(0.0f, inv_in.duty_v, 1.0f); //Limit Modulation at 100%
        Inverter_PWM_State.dc_w = MATHCALC__SATURATE_DIRECT(0.0f, inv_in.duty_w, 1.0f); //Limit Modulation at 100%
    }
    else if (Inverter_Carrier <= 0.0f) 
    {
        Inverter_Carrier = 0.0f;
        Inverter_Carrier_Dir = DIRECTION_UP;
    }

    //Get Desired States for Top Switches
    uint8 Su_des = pwm_compare(Inverter_PWM_State.dc_u, Inverter_Carrier);
    uint8 Sv_des = pwm_compare(Inverter_PWM_State.dc_v, Inverter_Carrier);
    uint8 Sw_des = pwm_compare(Inverter_PWM_State.dc_w, Inverter_Carrier);

    //Add deadtime to the top switches
    dt_insert_real(Su_des, &Inverter_PWM_State.Su_top, &Inverter_PWM_State.dt_u);
    dt_insert_real(Sv_des, &Inverter_PWM_State.Sv_top, &Inverter_PWM_State.dt_v);
    dt_insert_real(Sw_des, &Inverter_PWM_State.Sw_top, &Inverter_PWM_State.dt_w);

    //Calculate bottom switches
    Inverter_PWM_State.Su_bot = complementary_pwm(Inverter_PWM_State.Su_top);
    Inverter_PWM_State.Sv_bot = complementary_pwm(Inverter_PWM_State.Sv_top);
    Inverter_PWM_State.Sw_bot = complementary_pwm(Inverter_PWM_State.Sw_top);

    //Calculate Node Voltage (vx-gnd)
    va = phase_voltage(Inverter_PWM_State.Su_top, Inverter_PWM_State.Su_bot, inv_in.vdc);
    vb = phase_voltage(Inverter_PWM_State.Sv_top, Inverter_PWM_State.Sv_bot, inv_in.vdc);
    vc = phase_voltage(Inverter_PWM_State.Sw_top, Inverter_PWM_State.Sw_bot, inv_in.vdc);

    //Calc Vn (Motor Virtual Neutral)
    vn = (va + vb + vc) / 3.0f;

    //Get Vxn voltages (phase voltage vx-gnd - vn)
    v.Va = va - vn;
    v.Vb = vb - vn;
    v.Vc = vc - vn;

    /* ******* Optional for thermal analysis ******* */
    /*float Pcond =
        (INVERTER_VCE_SAT + INVERTER_RES_EQ * fabsf(inv_in.i_u)) * fabsf(inv_in.i_u) +
        (INVERTER_VCE_SAT + INVERTER_RES_EQ * fabsf(inv_in.i_v)) * fabsf(inv_in.i_w) +
        (INVERTER_VCE_SAT + INVERTER_RES_EQ * fabsf(inv_in.i_w)) * fabsf(inv_in.i_w);


    
    float Psw = (inv->Eon + inv->Eoff) * (fabsf(inv_in.i_u) + fabsf(inv_in.i_v) + fabsf(inv_in.i_w)) * (1.0f / Inverter_Tpwm);
    float P_loss = Pcond + Psw;*/

    return v;
}




    /* ********************************************* */
    /* ************  Auxiliary Functions  ********** */ 
    /* ********************************************* */

PhaseVoltages inverter_model_deadtime(InverterInput inv_in)
{
    float32 vdc, temp;
    PhaseVoltages v;

    //Apply Deadtime to the duty cycle signals
    float32 Da_eff = apply_deadtime(inv_in.duty_u, inv_in.i_u);
    float32 Db_eff = apply_deadtime(inv_in.duty_v, inv_in.i_v);
    float32 Dc_eff = apply_deadtime(inv_in.duty_w, inv_in.i_w);

    //Calc Vn (Motor Virtual Neutral)
    temp = (Da_eff + Db_eff + Dc_eff) / 3.0f;

    vdc = inv_in.vdc;

    //Get Vxn voltages (phase voltage vx-gnd - vn)
    v.Va = vdc * (Da_eff - temp);
    v.Vb = vdc * (Db_eff - temp);
    v.Vc = vdc * (Dc_eff - temp);

    return v;
}




float32 apply_deadtime(float32 D, float32 i_phase)
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

    if (i_phase > 0.0f)
        return D - Inverter_DT_Tpwm;   // Reduce mean voltage
    else if (i_phase < 0.0f)
        return D + Inverter_DT_Tpwm;   // increase mean voltage
    else
        return D; //do nothing for i=0
}



uint8 pwm_compare(float32 duty, float32 carrier)
{
    return (duty >= carrier) ? HIGH : LOW;
}




float32 phase_voltage(uint8 top, uint8 bot, float32 Vdc)
{
    if (top == HIGH && bot == LOW)
    {
        return Vdc;
    }
    else
    {
        return 0.0f; //If bottom is ON or at deadtime event
    }
}




void dt_insert_real(uint8 desired, uint8 *sx_top, float32 *dt_x)
{
    #ifdef SIMULATE_DEADTIME_EFFECT_PWM_INVERTER
    /* To be Implemented... 
        Require low simulation steps
        makes simulation slow
    */
    #else //Deadtime Effect Disabled (Load Top Commands w/ desired Duty Cycle)
    *sx_top = desired;
    *dt_x = 0.0f;
    #endif
}




uint8 complementary_pwm(uint8 sx_top)
{
    uint8 temp;

    if(sx_top == HIGH)
    {
        temp = LOW;
    }
    else if(sx_top == LOW)
    {
        temp = HIGH;
    }
    else
    {
        temp = 255; //Fault Condition - Should not be triggered
    }

    return temp;
}



float32 Inverter_Get_Data(uint8 index)
{
    float32 data = 0.0f;

    switch(index)
    {
        case 0:
            data = (float32)Inverter_Carrier;    
        break;

        case 1:
            data = (float32)Inverter_PWM_State.Su_top;
        break;

        case 2:
            data = (float32)Inverter_PWM_State.Su_bot;
        break;
    }

    return data;
}