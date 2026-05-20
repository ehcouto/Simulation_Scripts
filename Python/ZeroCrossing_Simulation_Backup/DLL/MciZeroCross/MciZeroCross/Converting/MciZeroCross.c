#include "stdafx.h"  //Remove it at embedeed code!

/* Code Starts Here! */

/**
*  @file		MciZeroCross.c file
*  @brief       Zero Crossing algorithm
*  @details     This module implements the zero crossing algorithm for input voltage applied to the single phase motor.
*  @author      Eduardo Henrique Couto - SW Motor Control Eng.
*  $Header:     MciZeroCross.c 1.00 2017/07/26 17:31:02CET Eduardio H Couto (Henrie2) Exp  $
* @copyright Copyright 2017 - $Date: 2017/07/26 17:31:02CET $. Whirlpool Corporation. All rights reserved � CONFIDENTIAL
*/
/*
*
*---------------------------------------------------------------------------------------------------------------------
*---------------------------------------------------------------------------------------------------------------------
*/
//-------------------------------------- Include Files ----------------------------------------------------------------
#include "MciZeroCross.h"


//-------------------------------------- PUBLIC (Variables) -----------------------------------------------------------

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------


//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
sint32 Filter_Data_Output[BUTTER_FILTER_OUTPUT];
sint32 Filter_Data_Input[BUTTER_FILTER_INPUT];
sint32 Sum_Den;
sint32 Sum_Num;
sint32 Y_Out;
sint32 Negative_Threshold_Lobe;
sint32 Positive_Threshold_Lobe;
MCI_ZC_STEP_TYPE Mci_Zc_Step_State;

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================



//---------------------------------------------------------------------------------------------------------------------
/**
*  @brief      Initialize module.
*  @details    This routine initializes main variables used in algorithm;
*
*
*  @param[in]
*  @param[out]   
*  @return		BOOL_TYPE
*/
void MciZeroCross__Initialize(void)
{
	sint32 i;
	Y_Out = 0;

	Gpio__PinConfig(PORT_MCI_ZEROCROSS, PIN_MCI_ZEROCROSS, TYPE_MCI_ZEROCROSS);

	i = BUTTER_FILTER_DEN - 1;
	for (i; i >= 0; i--)
	{
		Filter_Data_Output[i] = 0;
	}

	i = BUTTER_FILTER_NUM - 1;
	for (i; i >= 0; i--)
	{
		Filter_Data_Input[i] = 0;
	}

	Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_POS_SIGNAL;
	Negative_Threshold_Lobe = FILTER_COEFF_TO_IQ(500);
	Positive_Threshold_Lobe = FILTER_COEFF_TO_IQ(-442);
}




//---------------------------------------------------------------------------------------------------------------------
/**
*  @brief      ZeroCrossing algorithm
*  @details    Algorithm uses a butterworth filter and square wave signal generated from input zerocrossing HW to detect exact moment of the voltage zero crossing
*
*
*  @param[in]  ZeroCrossing HW signal
*  @param[out] 
*  @return     ZC event.
*/
sint32 MCiZeroCross__ButterworthFilter(void)
{
	sint32 tempS32;
	sint64 tempS32_1;
	sint32 zc_event;

	sint32 i;
	i = 0; //Initializing index
	Sum_Den = 0;
	Sum_Num = 0;

	Filter_Data_Input[0] = MciZeroCross__GetSignal();
	Filter_Data_Output[0] = Y_Out;


	for (i = 0; i < BUTTER_FILTER_DEN; i++)
	{
#if(1)
		tempS32_1 = (sint64)((sint64)Coeff_Butterworth_Den[i] * Filter_Data_Output[i]);
		tempS32 = (sint32)(tempS32_1 >> IQ_FILTER);
		Sum_Den += tempS32;
#else
		sint32 tempS32_2;
#define DIVISION_1   15
#define DIVISION_2   19 - DIVISION_1
		tempS32 = (sint32)(Filter_Data_Output[i] >> DIVISION_1);
		tempS32_2 = (sint32)(Coeff_Butterworth_Den[i] * tempS32);
		tempS32 = (sint32)(tempS32_2 >> DIVISION_2);
		Sum_Den += tempS32;
#endif
	}

	//tempS32 = 0;
	for (i = 0; i < BUTTER_FILTER_NUM; i++)
	{
		tempS32 = Coeff_Butterworth_Num[i] * Filter_Data_Input[i];
		Sum_Num += tempS32;
	}

	Y_Out = Sum_Num - Sum_Den; //Updating output

	i = BUTTER_FILTER_DEN - 1;
	for (i; i >= 1; i--)
	{
		Filter_Data_Output[i] = Filter_Data_Output[i - 1];
	}

	i = BUTTER_FILTER_NUM - 1;
	for (i; i >= 1; i--)
	{
		Filter_Data_Input[i] = Filter_Data_Input[i - 1];
	}

	// Check event

	zc_event = 0;

	switch (Mci_Zc_Step_State)
	{
	case MCI_ZC_POS_SLOPE_POS_SIGNAL:
		if (Y_Out <= Negative_Threshold_Lobe)
		{
			Mci_Zc_Step_State = MCI_ZC_NEG_SLOPE_POS_SIGNAL;
		}

		break;

	case MCI_ZC_NEG_SLOPE_POS_SIGNAL:
		if (Y_Out >= Negative_Threshold_Lobe)
		{
			zc_event = MCI_ZEROCROSSING_EVENT();
			Mci_Zc_Step_State = MCI_ZC_NEG_SLOPE_NEG_SIGNAL;
		}

		break;

	case MCI_ZC_NEG_SLOPE_NEG_SIGNAL:
		if (Y_Out >= Positive_Threshold_Lobe)
		{
			Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_NEG_SIGNAL;
		}

		break;

	case MCI_ZC_POS_SLOPE_NEG_SIGNAL:
		if (Y_Out <= Positive_Threshold_Lobe)
		{
			zc_event = MCI_ZEROCROSSING_EVENT();
			Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_POS_SIGNAL;
		}

		break;
	}


	return(zc_event);
}





//---------------------------------------------------------------------------------------------------------------------
/**
*  @brief       Get zero crossing input
*  @details     
*
*
*  @param[in]  
*  @param[out]
*  @return     
*/
sint32 MciZeroCross__GetSignal(void)
{
	sint32 signal_output;
	sint32 tempS32;
	tempS32 = Gpio__PinRead(PORT_MCI_ZEROCROSS, PIN_MCI_ZEROCROSS);
	

	if (tempS32 > 0)
	{
		signal_output = ZEROCROSS_POSITIVE_VALUE;
	}
	else
	{
		signal_output = ZEROCROSS_NEGATIVE_VALUE;
	}

	return(signal_output);
}