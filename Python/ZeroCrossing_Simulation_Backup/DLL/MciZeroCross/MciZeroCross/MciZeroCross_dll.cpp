#include "stdafx.h"
#include "MciZeroCross.h"
#include "Gpio.h"

#define EXP_DLL extern "C" __declspec(dllexport)

EXP_DLL sint32 New_ZC_Init(void);
EXP_DLL sint32 New_ZC_Filter(sint32);
//EXP_DLL sint32 New_ZC_GetOutput(sint32);

/* Dll code inside here! */
EXP_DLL sint32 New_ZC_Init(void)
{
	sint32 temp_S32;
	MciZeroCross__Initialize();
	temp_S32 = 1;
	return(temp_S32);
}

EXP_DLL sint32 New_ZC_Filter(sint32 input)
{
	sint32 temp_S32;

	Gpio__PinUpdate(input);
	temp_S32 = MCiZeroCross__ButterworthFilter();
	return(temp_S32);
}