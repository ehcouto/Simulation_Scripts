/**
 *  @file       
 *
 *  @brief      Basic description of file contents
 *
 *  @section    Applicable_Documents
 *					List here all the applicable documents if needed. <tr>	
 *
 *  $Header: $
 *
 *  @copyright  Copyright 2017-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef SOURCE_PROJECTSPECIFIC_MCI_MCIZEROCROSS_PRV_H_
#define SOURCE_PROJECTSPECIFIC_MCI_MCIZEROCROSS_PRV_H_

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================
const sint32 Coeff_Butterworth_Num[BUTTER_FILTER_NUM] =
{
    { FILTER_COEFF_TO_IQ(0.0000132937289) }, // 0
    { FILTER_COEFF_TO_IQ(0.0000531749156) }, // 1
    { FILTER_COEFF_TO_IQ(0.0000797623734) }, // 2
    { FILTER_COEFF_TO_IQ(0.0000531749156) }, // 3
    { FILTER_COEFF_TO_IQ(0.0000132937289) }, // 4
};

const sint32 Coeff_Butterworth_Den[BUTTER_FILTER_DEN] =
{
    { FILTER_COEFF_TO_IQ(-3.67172909) }, // 0
    { FILTER_COEFF_TO_IQ(5.067998390) }, // 1
    { FILTER_COEFF_TO_IQ(-3.11596693) }, // 2
    { FILTER_COEFF_TO_IQ(0.719910330) }, // 3
};

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================



#endif // SOURCE_PROJECTSPECIFIC_MCI_MCIZEROCROSS_PRV_H_


