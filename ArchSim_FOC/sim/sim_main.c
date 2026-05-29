/*
 * Simulation Architecture - Beko FOC Simulation
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

#define MOTOR_SPEED_RPM  2000.0f //rpm
#define MOTOR_RAMP_RPMS  500.0f //rpm

#define SIM_TIME_SEC     10.0f
#define FOC_FREQ_HZ      (uint32_t)MOTOR1_FAST_LOOP_FREQUENCY
#define SPEED_FREQ_HZ    (uint32_t)MOTOR1_SLOW_LOOP_FREQUENCY
#define MOTOR_FREQ_HZ    (uint32_t)(FOC_FREQ_HZ * 10U)

#define TS_SIM           (1.0f / MOTOR_FREQ_HZ)
#define TS_FOC           (1.0f / FOC_FREQ_HZ)
#define TS_SPEED         (1.0f / SPEED_FREQ_HZ)

#define VMAINS           230.0f
#define DCBUS_VOLTAGE    (VMAINS * sqrtf(2.0f) - 5.0f)
#define IPM_TEMPERATURE  25.0f

// --- Global Variables --- 
static uint32 foc_counter;
float32 sim_time;

PMSMParams Motor;
PMSMOutputs Motor_Out;

uint32_t MOTOR = MOTOR0;

mcInit_t mcInit;
mcMpvInit_t mcMpvInit1;

McSpeedReq_t Speed_Command;

static void system_init(void);
static void speed_command(void);

/* --- CSV File Pointer --- */
FILE *csv;

int main(void)
{
    real_t spd_ref, spd_rpm, iu, iv, iw, id, iq, torque;

    csv = fopen("sim_output.csv", "wb");
    fprintf(csv, "t,iu,iv,iw,SpeedRef,SpeedEst,id,iq,torque\n");

    //Variable Initialize
    foc_counter = 0;
    sim_time = 0.0;

    Speed_Command.mc_motorIndex = MOTOR;
    Speed_Command.mc_sprefmec = MOTOR_SPEED_RPM;
    Speed_Command.mc_ramp_duration = 0.01f; 
    Speed_Command.mc_rampin = MOTOR_RAMP_RPMS; //rpm/s       

    // Function Initialize
    //inverter_init(DCBUS_VOLTAGE);

    system_init();

    //Create the Motor 
    Motor.Rs = mcp_px[MOTOR]->phys.r; //25.4f; //ohm phase
    Motor.Ld = mcp_px[MOTOR]->phys.Ld; // H
    Motor.Lq = mcp_px[MOTOR]->phys.Lq; // H
    Motor.Phi = mcp_px[MOTOR]->phys.Phi; // Wb   
    Motor.p = mcp_px[MOTOR]->phys.pp; //pole pairs   
    Motor.J_Inv = 1.0f / 0.000006f;
    Motor.B = 0.00009f; 
    pmsm_initialize(Motor);


    //Run Loop
    while (sim_time < SIM_TIME_SEC)
    {
        // Loop de velocidade (FOC Hanlder)
        if((foc_counter % (uint32)(MOTOR_FREQ_HZ / FOC_FREQ_HZ)) == 0)
        {
            mcMxHandlerFL(MOTOR);
        }

        // Loop de velocidade (1 kHz)
        if((foc_counter % (uint32)(MOTOR_FREQ_HZ / SPEED_FREQ_HZ)) == 0)
        {
            mcMxHandlerSL(MOTOR);
            speed_command();

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

        //Run Inverter Model
        //inverter_step(0.0f, 0.0f, NULL, NULL, NULL);

        // Modelo do motor
        pmsm_step(mpv[MOTOR].v.vab.a, mpv[MOTOR].v.vab.b, 0.0f, TS_SIM);
        pmsm_get_outputs(&Motor_Out);
        brdSetData(Motor_Out.Iu, Motor_Out.Iv, Motor_Out.Iw, DCBUS_VOLTAGE, IPM_TEMPERATURE);
        
        spd_ref = mpv[MOTOR].v.spref;
        spd_rpm = mpv[MOTOR].v.spest;
        iu = Motor_Out.Iu;
        iv = Motor_Out.Iv;
        iw = Motor_Out.Iw;
        id = Motor_Out.Sts.id;
        iq = Motor_Out.Sts.iq;
        torque = Motor_Out.Te;

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
    brdSetData(0.0f, 0.0f, 0.0f, DCBUS_VOLTAGE, IPM_TEMPERATURE);
    
    mcInit.mcDrvTurnOnInrushRelay = brdTurnOnInrushRelay;
    mcInitMotorControl(&mcInit);

    mcMpvInit1.freqFL = MOTOR1_FAST_LOOP_FREQUENCY;
    mcMpvInit1.freqSL = MOTOR1_SLOW_LOOP_FREQUENCY;
    mcMpvInit1.mcBrdPwmEnable = brdPwmEnable;
    mcMpvInit1.mcBrdPwmDisable = brdPwmDisable;
    mcMpvInit1.mcBrdPwmEnablePhaseU = brdPwmEnablePhaseU;
    mcMpvInit1.mcBrdPwmEnablePhaseV = brdPwmEnablePhaseV;
    mcMpvInit1.mcBrdPwmEnablePhaseW = brdPwmEnablePhaseW;
    mcMpvInit1.mcBrdPwmEnablePhaseUVW = brdPwmEnablePhaseUVW;
    mcMpvInit1.mcBrdMapAdcChannels = brdMapAdcChannels;
    mcMpvInit1.mcBrdSampleCurrentsUVW = brdSampleCurrentsUVW;
    mcMpvInit1.mcBrdGetPhaseCurrentU = brdGetPhaseCurrentU;
    mcMpvInit1.mcBrdGetPhaseCurrentV = brdGetPhaseCurrentV;
    mcMpvInit1.mcBrdGetPhaseCurrentW = brdGetPhaseCurrentW;
    mcMpvInit1.mcBrdCurrentCalibrationInit = brdCurrentCalibrationInit;
    mcMpvInit1.mcBrdCurrentCalibrationHandler = brdCurrentCalibrationHandler;
    mcMpvInit1.mcBrdGetDcBusVoltage = brdGetDcBusVoltage;
    mcMpvInit1.mcBrdGetIpmTemperature = brdGetIpmTemperature;
    mcMpvInit1.mcBrdGetIpmFaultOutState = brdGetIpmFaultOutState;
    mcMpvInit1.mcBrdResetIpmFaultOutState = brdResetIpmFaultOutState;
    mcMpvInit1.mcBrdSetPwmDuties = brdSetPwmDuties;
    mcMpvInit1.mcBrdGetEncoderData = brdGetEncoderData;
    mcMpvInit1.mcBrdGetDCBusOpenStatus = brdGetDCBusOpenStatus;
    mcMpvInit1.mcBrdShortCircBotTransistors = brdShortCircuitBottomTransistors;
    mcAddMotor(&mcMpvInit1);

    mpv[MOTOR].control_state = INIT;
	mcEnableMotorControl(MOTOR);

    mcp_px[MOTOR] = &mcp_dw_circ_W20007335;
    fsp_px[MOTOR] = &fsp_dw_circ_W20007335;
}


/* ================= Speed Command Function =================== */
void speed_command(void)
{
    mcRequestSpeed(&Speed_Command);
}