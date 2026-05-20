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
 *  @copyright  Copyright 2015-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef BROWNOUT_PRM_H_
#define BROWNOUT_PRM_H_
#include "Hbl.h"
#include "NVData.h"
#include "Win.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief   - Checks if Brownout/Recover source value is conisdered valid
 *    @details - This could be e.g. Sequence ID associated to a LLI/GI used as source is not 0 -> reference signal value has been read properly
 *
 *    @param  - For the valid signal reference condition the return value from the connecting function shall be non zero value
 *              For the in signal reference condition the return value from the connecting function shall be zero value
 *				Default is TRUE - brownout / recover source is always considered valid.
 *
 *    @note  -  when Zero Cross Signal is used for the brown out condition detection
 *              "#define BROWNOUT__IS_VALID_SOURCE()                 (Hbl__GetGISequenceIDByIndex(Zc_Gi_Index) != 0)"
 */
#define BROWNOUT__IS_VALID_SOURCE()					(Hbl__GetGISequenceIDByIndex(Hbl__GetGIIndexByGIID(HBL_GI_RTN_LINE)) > 0)


/**
 *    @brief   - Connect the Brown Out detection source
 *    @details - This could be either -
 *               Zero Cross Signal,
 *               Voltage monitor circuit
 *
 *    @param  - For the valid brown out condition the return value from the connecting function shall be non zero value
 *              For the in valid brown out condition the return value from the connecting function shall be zero value
 *
 *    @note  -  when Zero Cross Signal is used for the brown out condition detection
 *    			"#define BROWNOUT_DETECTION_SOURCE()                 (!Hbl__IsZeroCrossPresent())"
 */
#define BROWNOUT_DETECTION_SOURCE()					(Hbl__GetGIDataConvertedByIndex(Hbl__GetGIIndexByGIID(HBL_GI_RTN_LINE)) <= 950)

//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief   - Connect the Brown Out recover source
 *    @details - This could be either -
 *               Zero Cross Signal,
 *               Voltage monitor circuit
 *
 *    @param  - For the valid brown out condition the return value from the connecting function shall be non zero value
 *              For the in valid brown out condition the return value from the connecting function shall be zero value
 *
 *    @note  -  when Zero Cross Signal is used for the brown out condition detection
 *              "#define BROWNOUT_RECOVER_SOURCE()                 (Hbl__IsZeroCrossPresent())"
 */
#define BROWNOUT_RECOVER_SOURCE()					(Hbl__GetGIDataConvertedByIndex(Hbl__GetGIIndexByGIID(HBL_GI_RTN_LINE)) > 1100)

//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief   - Detection Debounce filter time (in ms)
 *    @details - This parameter set a debounce time where BROWNOUT_DETECTION_SOURCE() condition goes TRUE and stays for this time
 *
 *    @param  - To set a debounce filter time set a value > 0
 *              To disable debounce filter feature set value to 0
 *				Default is 0 - no debounce filter configured.
 *
 */
// #define BROWNOUT_DETECTION_FILTER_MS             0

//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief   - Recover Debounce filter time (in ms)
 *    @details - This parameter set a debounce time where BROWNOUT_RECOVER_SOURCE() condition goes TRUE and stays for this time
 *
 *    @param  - To set a debounce filter time set a value > 0
 *              To disable debounce filter feature set value to 0
 *				Default is 0 - no debounce filter configured.
 *
 */
#define BROWNOUT_RECOVER_FILTER_MS					300

//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief   - Define the actions to be performed when the Brown out is detected
 *    @details - The power discharge time can be limited by turning off the loads and turning off the peripherals which consume more power
 *               When Brown out is detected its important to store the non volatile data and also
 *
 *    @param  -
 *
 *    @note  -  Store the non volatile data if the brown out is detected
 *    			"#define BROWNOUT_DETECTED_ACTIONS()                 NVData__BronwOut(TRUE)"
 */
#define BROWNOUT_DETECTED_ACTIONS()                 { Hbl__KeepLoadsOff(TRUE); \
													  NVData__BronwOut(TRUE); \
                                                      if (Win__GetBusStatus(WIN_BUS_STATUS_BREAK_OUT) == TRUE) \
                                                      { Win__StopBreak(); } \
                                                    }

//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief   - Define the actions to be performed when the power is recovered
 *    @details -
 *
 *    @param  -
 *
 *    @note  -  "#define BROWNOUT_RECOVERED_ACTIONS()                NVData__BronwOut(FALSE);
 *    "
 */
#define BROWNOUT_RECOVERED_ACTIONS()                { NVData__BronwOut(FALSE); \
													  Hbl__KeepLoadsOff(FALSE); \
													}

#endif // BROWNOUT_PRM_H_


