/*
 * drv.h
 *
 *  Created on: 10 Şub 2025
 *      Author: 26098107
 */

#ifndef COMMON_DRV_DRV_H_
#define COMMON_DRV_DRV_H_

#include "env.h"
#include "stdint.h"
#include <stdbool.h>


#define DRV_FREQ_FAST_LOOP      7550u
#define DRV_FREQ_SLOW_LOOP      1000
#define DRV_FREQ_WDG_TIMEOUT    1000


#define ADC_STEP					 0.0001007080078125f
#define CURRENT_AMP 				 1.36363636f
#define VOLTAGE_AMP				     0.007430f
#define VOLTAGE_AMP_3V3				 0.5f

#define RX_BUFFER_SIZE 12
#define TX_BUFFER_SIZE 23


#define NULL        ((void *)0)


typedef struct
{
    uint16_t   u2_iu_ad;
    uint16_t   u2_iv_ad;
    uint16_t   u2_iw_ad;
    uint16_t   u2_vdc_ad;
    uint16_t   u2_vphase_u_ad;
    uint16_t   u2_vphase_v_ad;
    uint16_t   u2_vphase_w_ad;
    uint16_t   u2_3v3_ad;
    uint16_t   u2_tempIPM_ad;
} r_mtr_adc_tb;


////////////////////////////////////////////////////////////////////////////////
// function declerations //

void drvInitMcu(void (*_McFuncFastLoop)(int32_t MxIndex),
                void (*_McFuncSlowLoop)(int32_t MxIndex),
                void (*drvFastFreqHandlerApp)(void),
                void (*drvSlowFreqHandlerApp)(void),
                void (*drvCommunicationIntHandlerApp)(void));
void drvDisableInterrupts(void);
void drvEnableInterrupts(void);
void drvWatchdogResetHandlerFL(void);
void drvResetWdg(void);
void drvPwmEnable(void);
void drvPwmDisable(void);
void drvPwmEnablePhaseU(void);
void drvPwmEnablePhaseV(void);
void drvPwmEnablePhaseW(void);
void drvPwmEnablePhaseUVW(void);
void drvSetPwmDuties(float dutyU, float dutyV, float dutyW);
void drvShortCircuitBottomTransistors(void);
int drvGetAdcWa(void);
int drvGetAdcV1(void);
int drvGetAdcV0(void);
int drvGetAdcUb(void);
void drvSampleAdcChannels(uint8_t sec);
void drvMapAdcChannels(uint8_t sec);
void drvSamplePhaseCurrents(void);
float drvGetDcBusVoltage(void);




// function declerations for the second motor //
void drvPwmEnable_M2(void);
void drvPwmDisable_M2(void);
void drvPwmEnablePhaseU_M2(void);
void drvPwmEnablePhaseV_M2(void);
void drvPwmEnablePhaseW_M2(void);
void drvPwmEnablePhaseUVW_M2(void);
void drvSetPwmDuties_M2(float dutyU, float dutyV, float dutyW);
void drvShortCircuitBottomTransistors_M2(void);
int drvGetAdcWa_M2(void);
int drvGetAdcV1_M2(void);
int drvGetAdcV0_M2(void);
int drvGetAdcUb_M2(void);
int drvGetAdcVa(void);
int drvGetAdcVa_M2(void);
void drvSampleAdcChannels_M2(uint8_t sec);
void drvMapAdcChannels_M2(uint8_t sec);
void drvSamplePhaseCurrents_M2(void);
float drvGetDcBusVoltage_M2(void);
void drvTurnOnInrushRelay_M2(void);
void drvTurnOffInrushRelay_M2(void);
void drvSetTXHigh_M2(void);
void drvSetTXLow_M2(void);
uint32_t drvGetRXState_M2(void);

void drvFpuInit(void);
void drvWatchdogInit(void);
void drvWatchdogDisable(void);
void drvWatchdogEnable(void);
void drvWatchdogResetHandlerFL(void);
void drvResetWdg(void);

extern void (*drvHighFreqHandlerMx)(int32_t MxIndex);
extern void (*drvLowFreqHandlerMx)(int32_t MxIndex);
extern void (*drvApos2msIntHandlerApp)(void);

void SVPWM_OL(uint16_t* PWMA, uint16_t* PWMB, uint16_t* PWMC);

void drvPumpSet(void);
void drvPumpReset(void);
void drvDispenserSet(void);
void drvDispenserReset(void);
void drvRegenSet(void);
void drvRegenReset(void);
void drvH2OSet(void);
void drvH2OReset(void);
void drvDiverterSet(void);
void drvDiverterReset(void);
float drvGetTempIPM_M1(void);
float drvGetTempIPM_M2(void);
void drvRelayHeaterSet(void);
void drvRelayHeaterReset(void);
void drvSwitchHeaterSet(void);
void drvSwitchHeaterReset(void);
void shortCircuitControl(void);
void shortCircuitControl_M2(void);

// function declerations //
////////////////////////////////////////////////////////////////////////////////

extern float adcDcBusVoltage;
extern uint16_t rawDcBusVoltage;

extern uint16_t adcWa;
extern uint16_t adcVa;
extern uint16_t adcUb;

extern uint16_t adcWa_M2;
extern uint16_t adcVa_M2;
extern uint16_t adcVb_M2;
extern uint16_t adcUb_M2;

extern uint16_t adcOffsetWa;
extern uint16_t adcOffsetVa;
extern uint16_t adcOffsetVb;
extern uint16_t adcOffsetUb;

extern uint16_t adcOffsetWa_M2;
extern uint16_t adcOffsetVa_M2;
extern uint16_t adcOffsetVb_M2;
extern uint16_t adcOffsetUb_M2;

extern float currentUb;
extern float currentVb;
extern float currentVa;
extern float currentWa;

extern float currentUb_M2;
extern float currentVb_M2;
extern float currentVa_M2;
extern float currentWa_M2;

//extern uint8_t ipmFaultOutHandled;
extern uint8_t ipmFaultOutHandled_M1;
extern uint8_t ipmFaultOutHandled_M2;

extern float ipmTemperature_M1;
extern float ipmTemperature_M2;

extern uint8_t rxBuffer[RX_BUFFER_SIZE];
extern uint8_t txBuffer[TX_BUFFER_SIZE];

extern const uint8_t crcTable[];
extern volatile uint32_t tickCounter;


extern float rmsCircPhW;
extern volatile uint16_t adcValDiverter;
extern volatile float torqueCirc[4];
extern volatile float torqueDrain[4];
extern volatile float uart_temperature;
extern volatile uint16_t adcValSyncPin;



#endif /* COMMON_DRV_DRV_H_ */
