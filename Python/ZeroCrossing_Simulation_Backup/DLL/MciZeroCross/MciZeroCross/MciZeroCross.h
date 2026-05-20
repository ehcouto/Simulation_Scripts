

#ifndef MCIZEROCROSS_H_
#define MCIZEROCROSS_H_

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "C_Extensions.h"
#include "MciZeroCross_prm.h"
#include "Gpio.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================
typedef enum
{
	//MCI_ZC_INIT = 0,                          //!< 0
	MCI_ZC_POS_SLOPE_POS_SIGNAL = 0,            //!< 1
	MCI_ZC_NEG_SLOPE_POS_SIGNAL,                //!< 2
	MCI_ZC_NEG_SLOPE_NEG_SIGNAL,                //!< 3
	MCI_ZC_POS_SLOPE_NEG_SIGNAL,                //!< 4
} MCI_ZC_STEP_TYPE;


//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void MciZeroCross__Initialize(void);
sint32 MCiZeroCross__ButterworthFilter(void);

#endif // MCIZEROCROSS_H_
