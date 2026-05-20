/*
 *  @file
 *  @brief      Motor Control Interface
 *
 *  @details    One interface for any motor present in system
 *
 *  @Module_Owner{Maciej Kobuszewski}
 *  $Header: Mci.c 1.12 2015/12/08 10:33:46BRST Mariusz  Tluscik (TLUSCM) Exp  $
 *
 *  @copyright  Copyright 2014 - $Date: 2015/12/08 10:33:46BRST $  Whirlpool Corporation.  All rights reserved - CONFIDENTIAL.
 */

#ifndef MCIZEROCROSS_PRM_H_
#define MCIZEROCROSS_PRM_H_


//-------------------------------------- Include Files ----------------------------------------------------------------
#include "C_Extensions.h"

//=====================================================================================================================
//-------------------------------------- PRIVATE (Constants & Defines) -------------------------------
//=====================================================================================================================
#define BUTTER_FILTER_NUM       5
#define BUTTER_FILTER_DEN       4
#define BUTTER_FILTER_INPUT     5
#define BUTTER_FILTER_OUTPUT    4
#define IQ_FILTER               19
#define FILTER_COEFF_TO_IQ(x)   (sint32)(((float)(1<<(IQ_FILTER)) * x ))

#define MCI_ZEROCROSSING_EVENT()     200  //Put here code to be executed at zero crossing

#define PORT_MCI_ZEROCROSS     PORTB
#define PIN_MCI_ZEROCROSS      1
#define TYPE_MCI_ZEROCROSS     INPUT_PULLDOWN

#define ZEROCROSS_POSITIVE_VALUE     512
#define ZEROCROSS_NEGATIVE_VALUE     -ZEROCROSS_POSITIVE_VALUE

#endif // MCIZEROCROSS_PRM_H_
