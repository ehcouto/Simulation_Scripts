/**
 *  @file
 *
 *  @brief      API of the Utilities modules which is a collection of commonly used operations with:\n
 *              Merging Bytes\n
 *              Manipulating Bits in arrays.
 *
 *  @section    Applicable_Documents
 *
 *
 *  $Header: Utilities.h 1.1 2015/03/17 16:33:04EDT MANOEL GARBUIO DE SOUZA (SOUZAMG) Exp  $
 *
 *  @copyright  Copyright 2011-$Date: 2015/03/17 16:33:04EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------



/**
 *  @file       Utilities.h
 *  @brief      Basic description of file contents
 *
 *---------------------------------------------------------------------------------------------------------------------
 *------------------- Copyright 2011.  Whirlpool Corporation.  All rights reserved - CONFIDENTIAL ---------------------
 *---------------------------------------------------------------------------------------------------------------------
 */
#ifndef UTILITIES_H_
#define UTILITIES_H_
#include "C_Extensions.h"
//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================


//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
uint32 Utilities__ConvertArrayTo32bits(uint8 * array);
uint32 Utilities__ConvertArrayTo24bits(uint8 * array);
uint16 Utilities__ConvertArrayTo16bits(uint8 * array);
uint16 Utilities__Merg2BytesTo16bits(uint8 msb, uint8 lsb);

void Utilities__SetBitInArray(uint8 * array, uint8 bit, uint8 value);
void Utilities__ToggleBitInArray(uint8 * array, uint8 bit);
uint8 Utilities__GetBitInArray(uint8 * array, uint8 bit);
uint16 Utilities__CountOneBitsInArray(uint8 * array, uint8 size);

#endif // UTILITIES_H_


