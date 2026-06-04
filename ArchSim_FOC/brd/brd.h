#ifndef BRD_H
#define BRD_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MOTOR0          0

void brd_init(void);
void brdPwmEnablePhaseU(void);
void brdPwmEnablePhaseV(void);
void brdPwmEnablePhaseW(void);
void brdPwmEnablePhaseUVW(void);
void brdSetPwmDuties(float dutyU, float dutyV, float dutyW);
void brdTurnOnInrushRelay(void);
void brdTurnOffInrushRelay(void);
void brdMapAdcChannels(uint8_t sec);
void brdSampleCurrentsUVW(uint8_t sec);
float brdGetPhaseCurrentU(void);
float brdGetPhaseCurrentV(void);
float brdGetPhaseCurrentW(void);
float brdGetDcBusVoltage(void);
bool brdCurrentCalibrationInit(void);
bool brdCurrentCalibrationHandler(void);
float brdGetIpmTemperature(void);
bool brdGetIpmFaultOutState(void);
void brdResetIpmFaultOutState(void);
void brdPwmEnable(void);
void brdPwmDisable(void);
void brdWatchdogResetHandlerFL(void);
void brdResetWdg(void);
void brdEnableInterrupts(void);
void brdDisableInterrupts(void);
bool brdGetDCBusOpenStatus(void);
bool brdGetEncoderData(float *_pos, float *_sp);
void brdShortCircuitBottomTransistors(void);

void brdGetData(float *current_u, float *current_v, float *current_w, float *dc_bus_voltage, float *ipm_temperature);
void ADC_Reading(void);

#endif