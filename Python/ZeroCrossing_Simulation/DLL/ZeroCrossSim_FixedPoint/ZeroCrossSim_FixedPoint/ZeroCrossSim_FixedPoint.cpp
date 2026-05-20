#include "stdafx.h"

#define EXP_DLL extern "C" __declspec(dllexport)

#define BUTTER_FILTER_NUM       5
#define BUTTER_FILTER_DEN       4
#define BUTTER_FILTER_INPUT     5
#define BUTTER_FILTER_OUTPUT    4
#define IQ_FILTER               19
#define FILTER_COEFF_TO_IQ(x)   (sint32)(((float)(1<<(IQ_FILTER)) * x ))

#define MCI_ZEROCROSSING_EVENT()     200  //Put here code to be executed at zero crossing

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


typedef enum
{
	//MCI_ZC_INIT = 0,                          //!< 0
	MCI_ZC_POS_SLOPE_POS_SIGNAL = 0,            //!< 1
	MCI_ZC_NEG_SLOPE_POS_SIGNAL,                //!< 2
	MCI_ZC_NEG_SLOPE_NEG_SIGNAL,                //!< 3
	MCI_ZC_POS_SLOPE_NEG_SIGNAL,                //!< 4
} MCI_ZC_STEP_TYPE;

sint32 Filter_Data_Output[BUTTER_FILTER_OUTPUT];
sint32 Filter_Data_Input[BUTTER_FILTER_INPUT];
sint32 Sum_Den;
sint32 Sum_Num;
sint32 Y_Out;
sint32 Negative_Threshold_Lobe;
sint32 Positive_Threshold_Lobe;
MCI_ZC_STEP_TYPE Mci_Zc_Step_State;


/* Functions */
EXP_DLL sint32 New_ZC_Init(void);
EXP_DLL sint32 New_ZC_Filter(sint32);
sint32 MciZeroCross__Initialize(void);
sint32 MCiZeroCross__ButterworthFilter(sint32);
sint32 MCiZeroCross__GetOutput(sint32);

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

EXP_DLL sint32 New_ZC_Filter(sint32 input)
{
	sint32 temp_S32;
	sint32 temp_S32_1;

	temp_S32_1 = input;
	temp_S32 = MCiZeroCross__ButterworthFilter(temp_S32_1);
	return(temp_S32);
}

EXP_DLL sint32 New_ZC_GetOutput(sint32 input)
{
	sint32 temp_S32;
	sint32 temp_S32_1;

	temp_S32_1 = input;
	temp_S32 = MCiZeroCross__GetOutput(temp_S32_1);
	return(temp_S32);
}






/* Zero Crossing code */
BOOL_TYPE MciZeroCross__Initialize(void)
{
	sint32 i;
	Y_Out = 0;

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
	Negative_Threshold_Lobe = 525;
	Positive_Threshold_Lobe = -468;
	return(TRUE);
}


sint32 MCiZeroCross__ButterworthFilter(sint32 input)
{
	sint32 tempS32;
	sint32 tempS32_2;
	sint64 tempS32_1;
	sint32 i;
	i = 0; //Initializing index
	Sum_Den = 0;
	Sum_Num = 0;

	Filter_Data_Input[0] = input; //Gpio__PinRead(PORTxx,Pinyy)
	Filter_Data_Output[0] = Y_Out;


	for (i = 0; i < BUTTER_FILTER_DEN; i++)
	{
#if(1)
		tempS32_1 = (sint64)((sint64)Coeff_Butterworth_Den[i] * Filter_Data_Output[i]);
		tempS32 = (sint32)(tempS32_1 >> IQ_FILTER);
		Sum_Den += tempS32;
#else
		tempS32 = (sint32)(Filter_Data_Output[i] >> 6);
		tempS32_2 = (sint32)(Coeff_Butterworth_Den[i] * tempS32);
		tempS32 = (sint32)(tempS32_2 >> 13);
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

	return(Y_Out);
}


sint32 MCiZeroCross__GetOutput(sint32 input)
{
	sint32 tempS32;
	sint32 zc_event;

	zc_event = 0;
	tempS32 = input;

	switch (Mci_Zc_Step_State)
	{
	case MCI_ZC_POS_SLOPE_POS_SIGNAL:
		if (tempS32 <= Negative_Threshold_Lobe)
		{
			Mci_Zc_Step_State = MCI_ZC_NEG_SLOPE_POS_SIGNAL;
		}

		break;

	case MCI_ZC_NEG_SLOPE_POS_SIGNAL:
		if (tempS32 >= Negative_Threshold_Lobe)
		{
			zc_event = MCI_ZEROCROSSING_EVENT();
			Mci_Zc_Step_State = MCI_ZC_NEG_SLOPE_NEG_SIGNAL;
		}

		break;

	case MCI_ZC_NEG_SLOPE_NEG_SIGNAL:
		if (tempS32 >= Positive_Threshold_Lobe)
		{
			Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_NEG_SIGNAL;
		}

		break;

	case MCI_ZC_POS_SLOPE_NEG_SIGNAL:
		if (tempS32 <= Positive_Threshold_Lobe)
		{
			zc_event = MCI_ZEROCROSSING_EVENT();
			Mci_Zc_Step_State = MCI_ZC_POS_SLOPE_POS_SIGNAL;
		}

		break;
	}
	return(zc_event);
}