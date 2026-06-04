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


//Select Motor
//#include "DW_WP_W20007335.h"
#include "DW_WP_W11652801.h"
//#include "DW_DR_W11402566.h"
//#include "DW_DR_W11377410.h"


//Select Board
#include "DW_DEA801.h"


float t_array[]    = {0.1f, 6.0f};  //Time in Seconds
float spd_array[]  = {3000.0f, 2600.0f}; //Motor Speed in RPM

#define SIM_TIME_SEC     10.0f //Simulation Time in Seconds

#define FOC_FREQ_HZ      (uint32_t)MOTOR1_FAST_LOOP_FREQUENCY
#define SPEED_FREQ_HZ    (uint32_t)MOTOR1_SLOW_LOOP_FREQUENCY
#define MOTOR_FREQ_HZ    (uint32_t)(FOC_FREQ_HZ * 10U)

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

static void system_init(void);
static void speed_command(const float *t_array, const float *spd_array, uint32_t n_points, float t_now);
static void Packing_Data_Csv(void);
static void load_torque(void);

/* --- CSV File Pointer --- */
FILE *csv;

int main(void)
{
    /* --- Initialize CSV File Pointer --- */
    csv = fopen("sim_output.csv", "wb");
    fprintf(csv, "t,iu,iv,iw,SpeedRef,SpeedEst,id,iq,torque\n");

    /* --- Initialize Variables --- */
    foc_counter = 0;
    sim_time = 0.0f;
    t_l      = 0.0f;

    Speed_Ref.mc_motorIndex = MOTOR;
    Speed_Ref.mc_sprefmec = 0.0f; //rpm
    Speed_Ref.mc_ramp_duration = 0.01f; 
    Speed_Ref.mc_rampin = MOTOR_RAMP_RPMS; //rpm/s       

    /* --- Initialize Functions --- */
    //inverter_init(DCBUS_VOLTAGE);
    system_init();

    /* --- Initialize the Motor --- */
    Motor.Rs = RS; //ohm phase
    Motor.Ld = LD; // H
    Motor.Lq = LQ; // H
    Motor.Phi = LAMBDA_M; // Wb   
    Motor.p = POLE_PAIRS; //pole pairs   
    Motor.J_Inv = 1.0f / INERTIA_J;
    Motor.B = VISC_B; 
    pmsm_initialize(Motor);

    /* --- loop Begin --- */
    while (sim_time < SIM_TIME_SEC)
    {
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
            speed_command(t_array, spd_array, sizeof(t_array) / sizeof(float), sim_time);

            //Run Low Frequency Handler (Speed Loop)
            mcMxHandlerSL(MOTOR);

            //Plot Data at FOC Frequency
            #if PLOT_TYPE == 0
                Packing_Data_Csv();
            #endif
        }

        //Run Inverter Model
        //inverter_step(0.0f, 0.0f, NULL, NULL, NULL);

        // Modelo do motor
        load_torque();
        pmsm_step(mpv[MOTOR].v.vab.a, mpv[MOTOR].v.vab.b, t_l, TS_SIM);
        
        #if PLOT_TYPE == 2
            Packing_Data_Csv();
        #endif

        //Step up time
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
	mcEnableMotorControl(MOTOR);
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
    t_l = TL_MAX * (mpv[MOTOR].v.spref / MAX_SPEED);
}


/* ================= CSV Packing Function =================== */
void Packing_Data_Csv(void)
{
    real_t spd_ref, spd_rpm, iu, iv, iw, id, iq, torque;

    pmsm_get_outputs(&Motor_Out);
    
    spd_ref = mpv[MOTOR].v.spref;
    spd_rpm = mpv[MOTOR].v.spest;
    iu = Motor_Out.Iu;
    iv = Motor_Out.Iv;
    iw = Motor_Out.Iw;
    id = Motor_Out.Sts.id;
    iq = Motor_Out.Sts.iq;
    torque = t_l; //Motor_Out.Te;

    fprintf(csv, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            sim_time,
            iu,
            iv,
            iw,
            spd_ref,
            spd_rpm,
            id,
            iq,
            torque);
}