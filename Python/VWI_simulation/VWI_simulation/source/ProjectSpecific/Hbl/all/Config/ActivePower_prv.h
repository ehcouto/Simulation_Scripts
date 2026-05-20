/**
 *  @file       
 *
 *  @brief      Basic View Header File description
 *
 *  @section    Applicable_Documents
 *					List here all the applicable documents if needed. <tr>	
 *
 *  $Header: $
 *
 *  @copyright  Copyright 2022-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef _ACTIVEPOWER_PRV_H_
#define _ACTIVEPOWER_PRV_H_

#include "TimeCapture.h"
#include "MciZeroCross.h"
#include "TimerInterruptExt.h"
#include "Mcl_prv.h"
#include "Mcl.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================
//! Select Power Compensation Parameters
//! 0- Yoda2
//! 1- FINN/Vostok
#define POWER_COMPENSATION_PARAMS                   1

//! Voltage offset filtering aplha bits
#define ACTIVEPOWER_VOLTAGE_OFFSET_ALPHA_BITS       (15U)

//! Current offset filtering alpha bits
#define ACTIVEPOWER_CURRENT_OFFSET_ALPHA_BITS       (15U)

//! Initialize interruption for fast handler
#define ACTIVEPOWER_FAST_HANDLER_FREQUENCY_HZ        5000
#define ACTIVEPOWER_INIT_EXT_MODULES()               TimerInterruptExt__SetupPerInt(ACTIVEPOWER_FAST_HANDLER_FREQUENCY_HZ, 1, 0)


//! P2P Current and Voltage Config
#define P2P_CURRENT_MS                               22    //22ms
#define P2P_VOLTAGE_MS                               22    //22ms

#define ZEROCROSS_FUNCTION_CALLBACK()                (TC_EVENT_TYPE)MciZeroCross__ReadLastEvent()

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================

#endif // _ACTIVEPOWER_PRV_H_

