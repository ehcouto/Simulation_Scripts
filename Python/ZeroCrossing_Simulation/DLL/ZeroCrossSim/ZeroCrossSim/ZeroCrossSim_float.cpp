#include "stdafx.h"
#include "MciZeroCross.h"
#include "MciZeroCross_prv.h"

#define EXP_DLL extern "C" __declspec(dllexport)

//#define BUTTER_FILTER_NUM    5
//#define BUTTER_FILTER_DEN    4
//#define BUTTER_FILTER_INPUT  5
//#define BUTTER_FILTER_OUTPUT 4
//#define IQ_FILTER            19
//
//#define MCI_ZEROCROSSING_EVENT()     200.0f  //Put here code to be executed at zero crossing

//const float Coeff_Butter_4th_Num[BUTTER_FILTER_NUM] =
//{
//	{ 0.0000132937289 }, // 0
//	{ 0.0000531749156 }, // 1
//	{ 0.0000797623734 }, // 2
//	{ 0.0000531749156 }, // 3
//	{ 0.0000132937289 }, // 4
//};
//
//const float Coeff_Butter_4th_Den[BUTTER_FILTER_DEN] =
//{
//	{ -3.67172909 }, // 0
//	{  5.06799839 }, // 1
//	{ -3.11596693 }, // 2
//	{  0.71991033 }, // 3
//};


//typedef enum
//{
//	//MCI_ZC_INIT = 0,                          //!< 0
//	MCI_ZC_POS_SLOPE_POS_SIGNAL = 0,            //!< 1
//	MCI_ZC_NEG_SLOPE_POS_SIGNAL,                //!< 2
//	MCI_ZC_NEG_SLOPE_NEG_SIGNAL,                //!< 3
//	MCI_ZC_POS_SLOPE_NEG_SIGNAL,                //!< 4
//} MCI_ZC_STEP_TYPE;

//float Filter_Data_Output[BUTTER_FILTER_OUTPUT];
//float Filter_Data_Input[BUTTER_FILTER_INPUT];
//float Sum_Den;
//float Sum_Num;
//float Y_Out;
//float Negative_Threshold_Lobe;
//float Positive_Threshold_Lobe;
//MCI_ZC_STEP_TYPE Mci_Zc_Step_State;


/* Functions */
EXP_DLL sint32 New_ZC_Init(void);
EXP_DLL float New_ZC_Filter(float);


/*----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------*/

/* Dll code inside here! */
EXP_DLL sint32 New_ZC_Init(void)
{
	sint32 temp_S32;
	temp_S32 = MciZeroCross__Initialize();
	return(temp_S32);
}

EXP_DLL float New_ZC_Filter(float input)
{
	float temp_S32;
	float temp_S32_1;

	temp_S32_1 = input;
	temp_S32 = MCiZeroCross__ButterworthFilter(temp_S32_1);
	return(temp_S32);
}





/* Zero Crossing code */
BOOL_TYPE MciZeroCross__Initialize(void)
{
	Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_POS_SIGNAL;
	Negative_Threshold_Lobe = 525.0f;
	Positive_Threshold_Lobe = -468.0f;
	return(1);
}


float MCiZeroCross__ButterworthFilter(float input)
{
	float tempS32;
	sint32 i;
	float tempf_1;
	float tempf_2;
	float zc_event;

	i = 0; //Initializing index
	
	Filter_Data_Input[0] = input; //Gpio__PinRead(PORTxx,Pinyy)
	Filter_Data_Output[0] = Y_Out;
	
	Sum_Den = 0;
	for (i = 0; i < BUTTER_FILTER_DEN; i++)
	{
		tempS32 = Coeff_Butter_4th_Den[i] * Filter_Data_Output[i];
		//tempS32 >>= IQ_FILTER;
		Sum_Den += tempS32;
	}
	
	tempS32 = 0;
	Sum_Num = 0;
	for (i = 0; i < BUTTER_FILTER_NUM; i++)
	{
		tempS32 = Coeff_Butter_4th_Num[i] * Filter_Data_Input[i];
		Sum_Num += tempS32;
	}
	
	Y_Out = Sum_Num - Sum_Den;
	
	i = BUTTER_FILTER_DEN-1;
	for (i; i >= 1; i--)
	{
		Filter_Data_Output[i] = Filter_Data_Output[i - 1];
	}
	
	i = BUTTER_FILTER_NUM-1;
	for (i; i >= 1; i--)
	{
		Filter_Data_Input[i] = Filter_Data_Input[i - 1];
	}
	
	zc_event = 0;
	tempf_1 = Y_Out;

	switch (Mci_Zc_Step_State)
	{
	case MCI_ZC_POS_SLOPE_POS_SIGNAL:
		if (tempf_1 <= Negative_Threshold_Lobe)
		{
			Mci_Zc_Step_State = MCI_ZC_NEG_SLOPE_POS_SIGNAL;
		}

		break;

	case MCI_ZC_NEG_SLOPE_POS_SIGNAL:
		if (tempf_1 >= Negative_Threshold_Lobe)
		{
			zc_event = MCI_ZEROCROSSING_EVENT();
			Mci_Zc_Step_State = MCI_ZC_NEG_SLOPE_NEG_SIGNAL;
		}

		break;

	case MCI_ZC_NEG_SLOPE_NEG_SIGNAL:
		if (tempf_1 >= Positive_Threshold_Lobe)
		{
			Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_NEG_SIGNAL;
		}

		break;

	case MCI_ZC_POS_SLOPE_NEG_SIGNAL:
		if (tempf_1 <= Positive_Threshold_Lobe)
		{
			zc_event = MCI_ZEROCROSSING_EVENT();
			Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_POS_SIGNAL;
		}

		break;
	}
	return(zc_event);
}