/**
 *  @file
 *
 *  @brief      modules which is a collection of commonly used operations with:\n
 *              Merging Bytes\n
 *              Manipulating Bits in arrays.
 *
 *  @details    The function sin this file are independent of microcontroller, endianess
 *
 *
 *  $Header: Utilities.c 1.1 2015/03/17 16:33:04EDT MANOEL GARBUIO DE SOUZA (SOUZAMG) Exp  $
 *
 *  @copyright  Copyright 2011-$Date: 2015/03/17 16:33:04EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//-------------------------------------- Include Files ----------------------------------------------------------------
#include "C_Extensions.h"
#include "Utilities.h"
//-------------------------------------- PUBLIC (Variables) -----------------------------------------------------------

static const unsigned short BYTE_SET_MASK[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
static const unsigned short BYTE_CLR_MASK[] = {0xFE,0xFD,0xFB,0xF7,0xEF,0xDF,0xBF,0x7F};

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------


//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================
//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method extracts a 32bits unsigned long from an unsigned char array
 * @param array unsigned char * pointer to the source array
 * @return unsigned long resulting of the data extraction.
 */
uint32 Utilities__ConvertArrayTo32bits(uint8 * array)
{
	uint32 retval;
    retval = ((uint32)array[0]) << 24;
    retval += ((uint32)array[1]) << 16;
    retval += ((uint32)array[2]) << 8;
    retval += ((uint32)array[3]);
    return (retval);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method extracts a 24bits unsigned long from an unsigned char array
 * @param array unsigned char *pointer to the source array.
 * @return unsigned long resulting of the data extraction.
 */
uint32 Utilities__ConvertArrayTo24bits(uint8 * array)
{
	uint32 retval;
    retval = ((uint32)array[0]) << 16;
    retval += ((uint32)array[1]) << 8;
    retval += ((uint32)array[2]);
    return (retval);
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method merges 2 bytes into a unsigned short int 16bits
 * @param msb most significant byte
 * @param lsb least significant byte
 * @return unsigned short with it the result of the merging.
 */
uint16 Utilities__Merg2BytesTo16bits(uint8 msb, uint8 lsb)
{
	uint16 retval;
    retval = ((uint16)msb) << 8;
    retval += ((uint16)lsb);
    return (retval);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method extracts a 16bits unsigned short from an unsigned char array
 * @param array uint16 *pointer to the source array.
 * @return unsigned short resulting of the data extraction.
 */
uint16 Utilities__ConvertArrayTo16bits(uint8 * array)
{
    return (Utilities__Merg2BytesTo16bits(array[0],array[1]));
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set a bit within an unsigned char array
 * @details the bit would vary from 0 to 255 which means the array can be only 32bytes long
 * @param array pointer to the unsigned char array
 * @param bit bit in the array (0 to 255)
 * @param value value to be set in the bit (0 / 1)
 */
void Utilities__SetBitInArray(uint8 * array, uint8 bit, uint8 value)
{
    uint8 s_byte;
    uint8 s_bit;
    s_byte = bit / 8;
    s_bit = bit % 8;

    if (value)
    {
        array[s_byte] |= BYTE_SET_MASK[s_bit];
    }
    else
    {
        array[s_byte] &= BYTE_CLR_MASK[s_bit];
    }

}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set a bit within an unsigned char array
 * @details the bit would vary from 0 to 255 which means the array can be only 32bytes long
 * @param array pointer to the unsigned char array
 * @param bit bit in the array (0 to 255)
 * @return Value of the specific bit
 */
uint8 Utilities__GetBitInArray(uint8 * array, uint8 bit)
{
    if (array[bit / 8] & BYTE_SET_MASK[bit % 8])
    {
        return (TRUE);
    }
    else
    {
        return (FALSE);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to toggle a bit within an unsigned char array
 * @details the bit would vary from 0 to 255 which means the array can be only 32bytes long
 * @param array pointer to the unsigned char array
 * @param bit bit in the array (0 to 255)
 */
void Utilities__ToggleBitInArray(uint8 * array, uint8 bit)
{
    array[bit / 8] ^= BYTE_SET_MASK[bit % 8];
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method counts the number of bits with a value of one in an array
 * @param array unsigned char *pointer to the source array.
 * @param size number of bytes in the array.
 * @return The number of bits with a value of one in the array.
 */
uint16 Utilities__CountOneBitsInArray(uint8 * array, uint8 size)
{
    uint16 retval;
    uint8 byte_index;
    uint8 byte_value;
    uint8 bit_mask;

    retval = 0;
    for (byte_index = 0; byte_index < size; byte_index++)
    {
        byte_value = array[byte_index];
        for (bit_mask = 0x80; bit_mask > 0; bit_mask >>= 1)
        {
            if ((byte_value & bit_mask) > 0)
            {
                retval++;
            }
        }
    }
    return retval;
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================


