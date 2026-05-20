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
#ifndef SOURCE_CATEGORY_APPLICATION_APPLICATIONDISH_PROCESSES_RINSEAIDDISPENSE_RINSEAIDDISPENSE_H_
#define SOURCE_CATEGORY_APPLICATION_APPLICATIONDISH_PROCESSES_RINSEAIDDISPENSE_RINSEAIDDISPENSE_H_

#include "Process.h"
//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================


//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
PROCESS_RESULT_TYPE RinseAidDispense__Process(PROCESS_CONTROL_TYPE* control);
void RinseAidDispense__Initialize(void);

#endif // SOURCE_CATEGORY_APPLICATION_APPLICATIONDISH_PROCESSES_RINSEAIDDISPENSE_RINSEAIDDISPENSE_H_


