#include "MciZeroCross_prm.h"


typedef enum
{
	//MCI_ZC_INIT = 0,                          //!< 0
	MCI_ZC_POS_SLOPE_POS_SIGNAL = 0,            //!< 1
	MCI_ZC_NEG_SLOPE_POS_SIGNAL,                //!< 2
	MCI_ZC_NEG_SLOPE_NEG_SIGNAL,                //!< 3
	MCI_ZC_POS_SLOPE_NEG_SIGNAL,                //!< 4
} MCI_ZC_STEP_TYPE;



float Filter_Data_Output[BUTTER_FILTER_OUTPUT];
float Filter_Data_Input[BUTTER_FILTER_INPUT];
float Sum_Den;
float Sum_Num;
float Y_Out;
float Negative_Threshold_Lobe;
float Positive_Threshold_Lobe;
MCI_ZC_STEP_TYPE Mci_Zc_Step_State;



sint32 MciZeroCross__Initialize(void);
float MCiZeroCross__ButterworthFilter(float);
