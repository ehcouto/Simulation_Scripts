/*
 * Simulation Architecture (Software-in-the-Loop) - Beko FOC Simulation
 * Author: EDUARDO HENRIQUE COUTO
 * Requisit: C99, <math.h>
 * Date: May/2026
 *
 * Notes:
 *  - It is used with GCC compiler (Winlibs) and C99 standard.
 *  - download Winlibs for C99 from https://winlibs.com/ and add the include and lib paths to your project settings.
 *  - GCC Version 16.1.0 or Later is recommended for better C99 support.
 *  - Tasks.json file should be updated according to the installation path of Winlibs.
 *  - This code is a simplified simulation of a FOC control loop for a PMSM motor, using the provided motor parameters and control functions from the mc library.
 *  - The simulation runs for a defined time, applying a speed command to the motor and updating the motor state using a simple RK4 integration method for the motor model. The outputs are printed to the console for analysis.            
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "C_Extensions.h"
#include "foc.h"
#include "pmsm_rk4.h"
#include "inverter_model.h"
#include "mc.h"
#include "brd.h"

//Plot @ 1Khz       ---> 0
//Plot @ FOC Freq   ---> 1
//Plot @ Motor Freq ---> 2
#define PLOT_TYPE     1

//Ideal               ---> 0
//Deadtime + losses   ---> 1
//dT + losses + delay ---> 2
//Real PWM Model      ---> 3
#define INVERTER_TYPE  2


//Motor & Board Selection
#include "Motor_Sel.h"
#include "Board_Sel.h"


float t_array[]    = {0.01f, 8.1f, 14.0f};  //Time in Seconds
float spd_array[]  = {3500.0f, 1800.0f, 3000.0f}; //Motor Speed in RPM

#define ARARY_SPD_SIZE   (uint32_t)(sizeof(t_array) / sizeof(float))

#define SIM_TIME_SEC     25.0f //Simulation Time in Seconds

#define FOC_FREQ_HZ      (uint32_t)MOTOR1_FAST_LOOP_FREQUENCY
#define SPEED_FREQ_HZ    (uint32_t)MOTOR1_SLOW_LOOP_FREQUENCY

#if INVERTER_TYPE == 3
    #define MOTOR_FREQ_HZ    (uint32_t)(FOC_FREQ_HZ * 50U) //Real PWM Simulation required more data points
#else
    #define MOTOR_FREQ_HZ    (uint32_t)(FOC_FREQ_HZ * 10U) 
#endif

#define TS_SIM           (1.0f / MOTOR_FREQ_HZ)
#define TS_FOC           (1.0f / FOC_FREQ_HZ)
#define TS_SPEED         (1.0f / SPEED_FREQ_HZ)

// --- Global Variables --- 
static uint32 foc_counter;
static float32 sim_time;

static PMSMParams Motor;
static PMSMOutputs Motor_Out;
 
static uint32_t MOTOR = MOTOR0;
 
static mcInit_t mcInit;
static mcMpvInit_t mcMpvInit;

static McSpeedReq_t Speed_Ref;
static float32 t_l;

//Inverter Variables
PhaseVoltages inverter_output;
InverterInput inverter_input;

static void system_init(void);
static void archsim_init(void);
static void speed_command(const float *t_array, const float *spd_array, uint32_t n_points, float t_now);
static void Packing_Data_Csv(void);
static void load_torque(void);
static void run_inverter_model(void);

/* --- CSV File Pointer --- */
FILE *csv;

int main(void)
{
    /* --- Initialize SIL Simulation --- */
    archsim_init();

    /* --- Initialize System --- */
    system_init();

    /* --- loop Begin --- */
    while (sim_time < SIM_TIME_SEC)
    {
        /* ********************************************* */
        /* ********** Load System Data ***************** */ 
        /* ********************************************* */
        pmsm_get_outputs(&Motor_Out);

        // Loop de velocidade (FOC Hanlder)
        if((foc_counter % (uint32)(MOTOR_FREQ_HZ / FOC_FREQ_HZ)) == 0)
        {
            //ADC Interruption Simulation
            ADC_Reading();

            //Run FOC Handler
            mcMxHandlerFL(MOTOR);

            //Plot Data at FOC Frequency
            #if PLOT_TYPE == 1
                Packing_Data_Csv();
            #endif 
        }

        // Loop de velocidade (1 kHz)
        if((foc_counter % (uint32)(MOTOR_FREQ_HZ / SPEED_FREQ_HZ)) == 0)
        {
            //Generate Speed Commands
            speed_command(t_array, spd_array, ARARY_SPD_SIZE, sim_time);

            // run load profile - generate t_l
            load_torque();

            //Run Low Frequency Handler (Speed Loop)
            mcMxHandlerSL(MOTOR);

            //Plot Data at FOC Frequency
            #if PLOT_TYPE == 0
                Packing_Data_Csv();
            #endif
        }

        // execute the inverter model
        run_inverter_model();

        // motor model
        pmsm_step(inverter_output.Va, inverter_output.Vb, inverter_output.Vc, t_l);
        
        #if PLOT_TYPE == 2
            Packing_Data_Csv();
        #endif

        //Step up time & counters
        sim_time += TS_SIM;
        foc_counter++;
    }

    printf("End of Simulation.\n");

    return 0;
}


/* ================= Auxiliary Functions =================== */

/* ================ System Init Functions =================== */
void system_init(void)
{
    brd_init();
    
    mcInit.mcDrvTurnOnInrushRelay = brdTurnOnInrushRelay;
    mcInitMotorControl(&mcInit);

    mcMpvInit.freqFL = MOTOR1_FAST_LOOP_FREQUENCY;
    mcMpvInit.freqSL = MOTOR1_SLOW_LOOP_FREQUENCY;
    mcMpvInit.mcBrdPwmEnable = brdPwmEnable;
    mcMpvInit.mcBrdPwmDisable = brdPwmDisable;
    mcMpvInit.mcBrdPwmEnablePhaseU = brdPwmEnablePhaseU;
    mcMpvInit.mcBrdPwmEnablePhaseV = brdPwmEnablePhaseV;
    mcMpvInit.mcBrdPwmEnablePhaseW = brdPwmEnablePhaseW;
    mcMpvInit.mcBrdPwmEnablePhaseUVW = brdPwmEnablePhaseUVW;
    mcMpvInit.mcBrdMapAdcChannels = brdMapAdcChannels;
    mcMpvInit.mcBrdSampleCurrentsUVW = brdSampleCurrentsUVW;
    mcMpvInit.mcBrdGetPhaseCurrentU = brdGetPhaseCurrentU;
    mcMpvInit.mcBrdGetPhaseCurrentV = brdGetPhaseCurrentV;
    mcMpvInit.mcBrdGetPhaseCurrentW = brdGetPhaseCurrentW;
    mcMpvInit.mcBrdCurrentCalibrationInit = brdCurrentCalibrationInit;
    mcMpvInit.mcBrdCurrentCalibrationHandler = brdCurrentCalibrationHandler;
    mcMpvInit.mcBrdGetDcBusVoltage = brdGetDcBusVoltage;
    mcMpvInit.mcBrdGetIpmTemperature = brdGetIpmTemperature;
    mcMpvInit.mcBrdGetIpmFaultOutState = brdGetIpmFaultOutState;
    mcMpvInit.mcBrdResetIpmFaultOutState = brdResetIpmFaultOutState;
    mcMpvInit.mcBrdSetPwmDuties = brdSetPwmDuties;
    mcMpvInit.mcBrdGetEncoderData = brdGetEncoderData;
    mcMpvInit.mcBrdGetDCBusOpenStatus = brdGetDCBusOpenStatus;
    mcMpvInit.mcBrdShortCircBotTransistors = brdShortCircuitBottomTransistors;
    
    set_motor(MOTOR);
    mcAddMotor(&mcMpvInit);
    
    mpv[MOTOR].control_state = INIT;

    //Setting Different Parameters for Simulation
    mpv[MOTOR].p.startup.ol.thalign_duration = PARAMETERS_STARTUP_OL_THALIGN_DURATION; /*In simulation long time for alignment is not required. 
                                                                                        keeping 100ms in simulation just to check signals & Controllers performance */
	
    //Enable Motor Control Module
    mcEnableMotorControl(MOTOR);
}




void archsim_init(void)
{
    /* --- Initialize CSV File Pointer --- */
    csv = fopen("sim_output.csv", "wb");
    fprintf(csv, "t,iu,iv,iw,SpeedRef,SpeedEst,SpeedReal,id,id_m,iq,iq_m,vd,vq,torque,tq_m,tq_load,dc_u,dc_v,dc_w\n");

    /* --- Initialize the Motor --- */
    Motor.Rs = RS; //ohm phase
    Motor.Ld = LD; // H
    Motor.Lq = LQ; // H
    Motor.Phi = LAMBDA_M; // Wb   
    Motor.p = POLE_PAIRS; //pole pairs   
    Motor.J_Inv = 1.0f / INERTIA_J;
    Motor.B = VISC_B; 

    /* --- Initialize Variables --- */
    foc_counter = 0;
    sim_time = 0.0f;
    t_l      = 0.0f;

     /* --- Initialize Speed Command Parameters --- */
    Speed_Ref.mc_motorIndex = MOTOR;
    Speed_Ref.mc_sprefmec = 0.0f; //rpm
    Speed_Ref.mc_ramp_duration = 0.01f; 
    Speed_Ref.mc_rampin = MOTOR_RAMP_RPMS; //rpm/s 

    inverter_model_init(TS_FOC, TS_SIM);
    
    /* --- Initialize PMSM Model Module --- */
    pmsm_initialize(Motor, TS_SIM);
}




/* ================= Speed Command Function =================== */
void speed_command(const float *t_array, const float *spd_array, uint32_t n_points, float t_now)
{
    float spd_ref = 0.0f;

    // Initialize at zero...
    if (t_now <= t_array[0])
        spd_ref = 0.0f;

    // saturate the speed at the last command...
    if (t_now >= t_array[n_points - 1])
        spd_ref = spd_array[n_points - 1];

    // seek correct interval...
    for (size_t i = 0; i < n_points - 1; i++)
    {
        if (t_now >= t_array[i] && t_now < t_array[i + 1])
        {
            spd_ref = spd_array[i];
        }
    }
   
    //Ref. Speed Udpate
    Speed_Ref.mc_sprefmec = spd_ref;

    //Report to MC
    mcRequestSpeed(&Speed_Ref);
}



/* ================= Load Torque Generatuion =================== */
void load_torque(void)
{
    float32 speed_rpm;

    speed_rpm = mpv[MOTOR].v.spref;

#if(LOAD_TYPE == LOAD_TYPE_EMPTY)
    t_l = 0.0f;
#elif((LOAD_TYPE == LOAD_TYPE_SPD_LINEAR))
    t_l = TL_MAX * (speed_rpm / MAX_SPEED);
#elif((LOAD_TYPE == LOAD_TYPE_SPD_QUADRATIC))
    t_l = TL_K_QD * (speed_rpm * speed_rpm);
#elif((LOAD_TYPE == LOAD_TYPE_HYDRAULIC_SIMPLE))
    t_l = K_HYD * (speed_rpm * speed_rpm);
#endif
}




void run_inverter_model(void)
{
    DutyCycleStates duties;

    duties = brdGetDuties();

    //Load Duties
    inverter_input.duty_u = duties.U;
    inverter_input.duty_v = duties.V;
    inverter_input.duty_w = duties.W;

    //Load Currents from motor model
    inverter_input.i_u = Motor_Out.Iu;
    inverter_input.i_v = Motor_Out.Iv;
    inverter_input.i_w = Motor_Out.Iw;

    //Load DC Bus Voltage
    inverter_input.vdc = brdGetDcBusVoltage();

    //Run the selected inverter model
    #if(INVERTER_TYPE == 0)
        inverter_output = inverter_model_ideal(inverter_input);
    #elif(INVERTER_TYPE == 1)
        inverter_output = inverter_model_dt_losses(inverter_input);  
    #elif(INVERTER_TYPE == 2)
        inverter_output = inverter_model_delay(inverter_input);   
    #elif(INVERTER_TYPE == 3)
        inverter_output = inverter_model_pwm(inverter_input);   
    #endif
}




/* ================= CSV Packing Function =================== */
void Packing_Data_Csv(void)
{
    real_t spd_ref, spd_rpm, spd_real, iu, iv, iw, id, id_m, iq, iq_m, torque, tq_load, tq_m, dc_u, dc_v, dc_w, vd, vq;

    spd_ref = mpv[MOTOR].v.spref;
    spd_rpm = mpv[MOTOR].v.spest;
    spd_real = Motor_Out.Sts.omega_m * (30.0 / MC_PI);
    iu = Motor_Out.Iu;
    iv = Motor_Out.Iv;
    iw = Motor_Out.Iw;
    id_m = Motor_Out.Sts.id;
    iq_m = Motor_Out.Sts.iq;
    id = mpv[MOTOR].v.idq.d;
    iq = mpv[MOTOR].v.idq.q;
    tq_m = Motor_Out.Te;
    torque = iq * mpv[MOTOR].p.phys.Kt * 0.70710678118654752440f;
    tq_load = t_l;
    dc_u = mpv[MOTOR].v.duvw_comp.u;
    dc_v = mpv[MOTOR].v.duvw_comp.v;
    dc_w = mpv[MOTOR].v.duvw_comp.w;
    vd = mpv[MOTOR].v.vdq.d;
    vq = mpv[MOTOR].v.vdq.q;

    fprintf(csv, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            sim_time,
            iu,
            iv,
            iw,
            spd_ref,
            spd_rpm,
            spd_real,
            id,
            id_m,
            iq,
            iq_m,
            vd,
            vq,
            torque,
            tq_m,
            tq_load,
            dc_u,
            dc_v,
            dc_w);
}

