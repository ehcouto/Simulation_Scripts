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

/* Filter Config: ButterWorth Band Pass 
	Low Cut Freq.   35Hz
	High Cut Freq.  85Hz

	Frequency:      5000Hz*/
const sint32 Coeff_Butterworth_Num[BUTTER_FILTER_NUM] =
{
	FILTER_COEFF_TO_IQ(0.0009446918438401511),  // 0
	FILTER_COEFF_TO_IQ(0.0000000000000000000),  // 1
	FILTER_COEFF_TO_IQ(-0.0018893836876803021),  // 2
	FILTER_COEFF_TO_IQ(0.0000000000000000000),  // 3
	FILTER_COEFF_TO_IQ(0.0009446918438401511),  // 4
};

const sint32 Coeff_Butterworth_Den[BUTTER_FILTER_DEN] =
{
	FILTER_COEFF_TO_IQ(-3.9020103730285000000),  // 0
	FILTER_COEFF_TO_IQ(5.7194171024348870000),  // 1
	FILTER_COEFF_TO_IQ(-3.7323614553986720000),  // 2
	FILTER_COEFF_TO_IQ(0.9149758348014336000),  // 3
};

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================



#endif // SOURCE_PROJECTSPECIFIC_MCI_MCIZEROCROSS_PRV_H_


