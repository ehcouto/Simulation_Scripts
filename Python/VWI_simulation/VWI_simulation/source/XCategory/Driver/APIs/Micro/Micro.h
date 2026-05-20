/**
 *  @file
 *  @defgroup   CLASS_B
 *
 *  @brief      This module provides standard API for Microcontroller.
 *
 *  $Header: Micro.h 1.6 2015/08/26 17:34:49EDT Dasar Hanumanth (DASARHB) Exp  $
 *
 *  @copyright  *****  Copyright 2011.  Whirlpool Corporation.  All rights reserved - CONFIDENTIAL  *****
 */
#ifndef MICRO_H_
    #define MICRO_H_
#include "C_Extensions.h"
#include "SRSystemConfig.h"
#include "uc.h"
#include "Micro_prm.h"
#include "Micro_defs.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief  - Defines the Enabling or disabling of the Safety relevant test features
 *    @details-
 *
 *    Below are the tests to be performed during boot and run-time -
 *    CPU Register test,
 *    Stack Test
 *    RAM Test(at boot time)
 *    Flash Test
 *    Watchdog Test
 *
 *    @param  - ENABLED(1)    - Which Enables the ClassB Test. ClassB software shall enable this macro
 *              DISABLED(0)   – Which Disables the ClassB Test. ClassA software can disables this macro
 *
 *    @note -   "#define MICRO_SR_FEATURE_TEST                   ENABLED"
 *              "#define MICRO_SR_FEATURE_TEST                   DISABLED"
 */
#if (CLASSB_FEATURE == ENABLED)
    #ifndef MICRO_SR_FEATURE_TEST
        #define MICRO_SR_FEATURE_TEST                               ENABLED
    #endif
#else
    #ifndef MICRO_SR_FEATURE_TEST
        #define MICRO_SR_FEATURE_TEST                               DISABLED
    #endif
#endif

#if (CLASSB_FEATURE == ENABLED)
    #if (MICRO_SR_FEATURE_TEST != ENABLED)
        SW05_WARNING(1, "MICRO_SR_FEATURE_TEST is disabled", SW05_CLASSB)
    #endif
#endif

typedef struct
{
    uint8 * id;
    uint8 id_size;
}MICRO_ID_TYPE;

typedef struct
{
    uint8 DEBBUGER      :1;
    uint8 LVD           :1;
    uint8 CLOCK         :1;
    uint8 EMC           :1;
    uint8 ILLEGAL_OP    :1;
    uint8 WATCHDOG      :1;
    uint8 EXTERNAL      :1;
    uint8 POR           :1;
}MICRO_LAST_RESET_TYPE;

typedef enum
{
    MICRO_LVD_DISABLED,
    MICRO_LVD_ENABLED_NOT_DETECTING,
    MICRO_LVD_ENABLED_DETECTING
}MICRO_LVD_STATUS_TYPE;

typedef enum
{
    MICRO_RESET_MODE_COLD,
    MICRO_RESET_MODE_WARM
}MICRO_RESET_MODE_TYPE;

//! Below actions are valid only if the reset is MICRO_RESET_MODE_WARM
typedef enum
{
    MICRO_WARM_RESET_ACTION_ENTER_LOWPOWER     = 0x01,     //! On next reset enter Low power mode
    MICRO_WARM_RESET_ACTION_ENTER_IAP          = 0x02,     //! On next reset enter IAP (In Application Programming) mode
    MICRO_WARM_RESET_ACTION_SKIP_CLASSB_TEST   = 0x04,     //! On next reset skip ClassB testing
    MICRO_WARM_RESET_ACTION_SKIP_WATCHDOG_TEST = 0x08,     //! On next reset skip watch dog test
    MICRO_WARM_RESET_ACTION_USER_CONFIG_1      = 0x10,     //! On next reset enter IAP (In Application Programming) mode
    MICRO_WARM_RESET_ACTION_USER_CONFIG_2      = 0x20,     //! On next reset enter IAP (In Application Programming) mode
    MICRO_WARM_RESET_ACTION_USER_CONFIG_3      = 0x40,     //! On next reset enter IAP (In Application Programming) mode
    MICRO_WARM_RESET_ACTION_USER_CONFIG_4      = 0x80,     //! On next reset enter IAP (In Application Programming) mode
}MICRO_WARM_RESET_ACTION_TYPE;

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
//! Safety Relevant Methods
void Micro__Initialize(void);
void Micro__ServiceWatchdog(void);

void Micro__SetLowSpeedMode(void);
void Micro__SetNormalSpeedMode(void);
void Micro__GoSleep(void);

INTERRUPT_DIRECTIVE void Micro__Trap(void);

//! Non Safety relevant Methods
MICRO_CLOCK_DEF * Micro__GetClock(void);
void Micro__DelayNumNops(uint16 delay);
uint32 Micro__GetFlashSize(void);
uint32 Micro__GetRamSize(void);
MICRO_ID_TYPE * Micro__GetCoreID(void);
MICRO_LAST_RESET_TYPE * Micro__GetLastResetstatus(void);

//! Methods only for Class B
INTERRUPT_DIRECTIVE void Micro__BootTest(void);
INTERRUPT_DIRECTIVE void Micro__IndependentClockISR(void);
void Micro__SRRamTest(void);
void Micro__SRFlashTest(void);

//! Low Voltage detector methods
void Micro__ConfigureLvd(void);
void Micro__ClearLvdIsrRegisters(void);
MICRO_LVD_STATUS_TYPE Micro__GetLvdStatus(void);

//! Reset Handling methods
void Micro__ForceReset(MICRO_RESET_MODE_TYPE reset_mode);
MICRO_RESET_MODE_TYPE Micro__GetResetMode(void);
uint8 Micro__GetLoopResetCount(void);
void Micro__ResetLoopResetCount(void);

//! Below API's are valid only if the reset is MICRO_RESET_MODE_WARM
void Micro__EnableWarmResetAction(MICRO_WARM_RESET_ACTION_TYPE warm_reset_action);
void Micro__DisableWarmResetAction(MICRO_WARM_RESET_ACTION_TYPE warm_reset_action);
MICRO_WARM_RESET_ACTION_TYPE Micro__GetWarmResetAction(void);
uint8 Micro__GetWarmResetCount(void);

#endif // MICRO_H_
