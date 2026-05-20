/**
 *  @file
 *
 *  @brief      Supervises state transitions among the different operation modes of the product:
 *                  Programming, Run, Delay, END, PAUSE and FAULT
 *
 *  @details

    @startuml{Delay_sq.png}
        hide footbox
        autonumber

        actor User
        participant HMI
        participant Regulations
        participant Status
        participant Supervisor

        loop 100 milliseconds (embOS AppMgr Task call rate)
            alt Supervisor_State == PROGRAMMING
                User -> HMI : Select Delay (Start/Stop) Time
                HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_HH, value)
                HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_MM, value)
                HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_SS, value)

                loop function = FUNC_ID_DELAY_HH, FUNC_ID_DELAY_MM and FUNC_ID_DELAY_SS
                    Supervisor -> Regulations : GetValue(function)
                    Regulations --> Supervisor: value = hours, minutes or seconds
                    Supervisor -> Supervisor : Delay_Time = hours * 3600 + minutes * 60 + seconds
                end

                alt
                    User -> HMI : Select Start
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_START, FS_START_CONDITION)
                    Supervisor -> Regulations : GetValue(FUNC_ID_START)
                    Regulations --> Supervisor: value = FS_START_CONDITION

                    alt Delay_Time == 0 && Selector != 0 && OK_TO_START
                        Supervisor -> Supervisor : eventStart
                        note right : Causes Supervisor to go to RUN state.
                    else Delay_Time != 0 && Selector != 0
                        Supervisor -> Supervisor : eventDelay
                        note right : Causes Supervisor to go to DELAY state.
                    end
                else
                    User -> HMI : Select Cancel
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_START, FS_RESET_CONDITION)
                    Supervisor -> Regulations : GetValue(FUNC_ID_START)
                    Regulations --> Supervisor: value = FS_RESET_CONDITION
                    Supervisor -> Supervisor : eventReset, Delay_Time = 0
                    note right : Supervisor stays in PROGRAMMING state.
                end

            else Supervisor_State == DELAY
                alt
                    User -> HMI : Select Start
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_START, FS_RESTART_CONDITION)
                    Supervisor -> Regulations : GetValue(FUNC_ID_START)
                    Regulations --> Supervisor: value = FS_RESTART_CONDITION
                    Supervisor -> Supervisor : [OK_TO_START] eventStart, set Delay_Time to 0
                    note right : Causes Supervisor to go to RUN state.
                else
                    User -> HMI : Select Cancel
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_START, FS_RESET_CONDITION)
                    Supervisor -> Regulations : GetValue(FUNC_ID_START)
                    Regulations --> Supervisor: value = FS_RESET_CONDITION
                    Supervisor -> Supervisor : eventReset, set Delay_Time to 0
                    note right : Causes Supervisor to go to PROGRAMMING state.
                else

                    User -> HMI : Select Delay (Start/Stop) Time
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_HH, value)
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_MM, value)
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_SS, value)
                    loop function = FUNC_ID_DELAY_HH, FUNC_ID_DELAY_MM and FUNC_ID_DELAY_SS
                        Supervisor -> Regulations : GetValue(function)
                        Regulations --> Supervisor: value = hours, minutes or seconds
                        Supervisor -> Supervisor : Delay_Time = hours * 3600 + minutes * 60 + seconds
                        note right : If Delay_Time set to 0, eventStart will be generated.
                    end
                else
                    User -> HMI : Set Pause
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_START, FS_PAUSE_CONDITION)
                    Supervisor -> Regulations : GetValue(FUNC_ID_START)
                    Regulations --> Supervisor: value = FS_PAUSE_CONDITION
                    Supervisor --> Supervisor: EventPause
                    note right : Causes Supervisor to go to DELAY_PAUSE state which reports PAUSE state and runs PAUSE cycle
                end
                Supervisor -> Supervisor : Decrement Delay_Time every 10 loops (1 second)
                Status -> HMI : Delay_Time
                alt Delay_Time == 0 && OK_TO_START
                    Supervisor -> Supervisor : eventStart
                    note right : Causes Supervisor to go to RUN state.
                end
            else Supervisor_State == DELAY_PAUSE
                Supervisor --> Supervisor : Run Pause Cycle and report PAUSE state
                alt
                    User -> HMI : Select Delay (Start/Stop) Time
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_HH, value)
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_MM, value)
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_DELAY_SS, value)
                    loop function = FUNC_ID_DELAY_HH, FUNC_ID_DELAY_MM and FUNC_ID_DELAY_SS
                        Supervisor -> Regulations : GetValue(function)
                        Regulations --> Supervisor: value = hours, minutes or seconds
                        Supervisor -> Supervisor : Delay_Time = hours * 3600 + minutes * 60 + seconds
                        note right : If Delay_Time set to 0, eventStart will be generated when pause is resumed.
                    end
                else
                    User -> HMI : Select Start
                    HMI -> Regulations : SetRequestedIndex(FUNC_ID_START, FS_START_CONDITION)
                    Supervisor -> Regulations : GetValue(FUNC_ID_START)
                    Regulations --> Supervisor: value = FS_START_CONDITION
                    alt OK_TO_START
                        Supervisor -> Supervisor : eventResume
                    end
                    note right : Causes Supervisor to go back to Delay state.
                end
            end
        end

    @enduml


    @startuml{Supervisor_SM.png}
        title Supervisor State Machine
        '' Trace = EVENT

        State PROGRAMMING
        State RUN
        State DELAY
        State END_OF_CYCLE
        State PAUSE
        State FAULT
        State DELAY_PAUSE

        ' Use State = Supervisor_Data[Current_Compartment].State
        [*] --> PROGRAMMING

        ' PROGRAMMING State
            PROGRAMMING: entry / ProgrammingEntry
            PROGRAMMING: do / ProgrammingDo()
            PROGRAMMING --> PROGRAMMING: evReset / Reset()
            PROGRAMMING --> RUN: evStart / Start()
            PROGRAMMING --> DELAY: evDelay
            PROGRAMMING --> FAULT: evFault

        ' RUN State
            RUN: entry / RunEntry
            RUN: do / RunDo()
            RUN: exit / RunExit()
            RUN --> END_OF_CYCLE  : evEnd
            RUN --> PAUSE  : evPause
            RUN --> PROGRAMMING: evReset / Reset() ResetTasks()
            RUN --> FAULT: evFault

        ' DELAY State
            DELAY: entry / DelayEntry
            DELAY: do / DelayDo()
            DELAY --> RUN: evStart / Start()
            DELAY --> PROGRAMMING: evReset / Reset()
            DELAY --> FAULT: evFault
            DELAY --> DELAY_PAUSE: evPause

        ' DELAY PAUSE State
            DELAY_PAUSE: entry / DelayPauseEntry
            DELAY_PAUSE: do / DelayPauseDo()
            DELAY_PAUSE: exit / DelayPauseExit()
            DELAY_PAUSE --> RUN: evStart / Start()
            DELAY_PAUSE --> PROGRAMMING: evReset / Reset()
            DELAY_PAUSE --> FAULT: evFault
            DELAY_PAUSE --> DELAY: evResume

        ' END_OF_CYCLE State
            END_OF_CYCLE: entry / EndEntry
            END_OF_CYCLE --> PROGRAMMING: evReset / Reset()
            END_OF_CYCLE --> FAULT: evFault

        ' PAUSE State
            PAUSE: entry / PauseEntry()
            PAUSE: do / PauseDo()
            PAUSE: exit / PauseExit()
            PAUSE --> RUN  : evResume / Resume()
            PAUSE --> PROGRAMMING: evReset / Reset() ResetTasks()
            PAUSE --> FAULT: evFault

        ' FAULT State
            FAULT: entry / FaultEntry()
            FAULT: do / FaultDo()
            FAULT --> PROGRAMMING: evFaultReset / Reset()
            FAULT --> FAULT: evFault

    @enduml
 *
 *  @copyright  Copyright 2015-2016 Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
// -- This Module --
#include "SystemConfig.h"
#include "Supervisor.h"
#include "Supervisor_prv.h"

// -- Other Modules --
#if (API012EXP_FEATURE == ENABLED)
    #include "API012Exp.h"
#endif

#include "ActivateTask.h"
#include "Estimator.h"
#include "Fault.h"
#include "Interpreter.h"
#include <limits.h>
#include "Log.h"
#include "Maintain.h"
#include "Micro.h"
#include "Mode.h"
#include "NVData.h"
#include "Regulations.h"
#include "Selector.h"
#include "SettingFile.h"
#include "Statistics.h"
#include "Variable.h"
#include <string.h>

//=====================================================================================================================
//=== StateEngineCrank (v2.8) Auto-Generated Definitions for Supervisor State Machine
//=====================================================================================================================
// !! StateEngineCrank cannot update this code if you change the header !!


//! Unique names used to identify states of the Supervisor state machine.
typedef enum SUPERVISOR_STATE_ENUM
{
    STATE_SUPERVISOR_NONE         = 0,

    STATE_SUPERVISOR_PROGRAMMING  = 1,              //!< (initial_state)
    STATE_SUPERVISOR_RUN          = 2,
    STATE_SUPERVISOR_DELAY        = 3,
    STATE_SUPERVISOR_END_OF_CYCLE = 4,
    STATE_SUPERVISOR_PAUSE        = 5,
    STATE_SUPERVISOR_FAULT        = 6,
    STATE_SUPERVISOR_DELAY_PAUSE  = 7,

    STATE_SUPERVISOR_END          = 8
} SUPERVISOR_STATE_TYPE;

// -- Private Function Prototypes used by the state machine for customization functions --
static void DelayDo(void);
static void DelayEntry(void);
static void DelayPauseDo(void);
static void DelayPauseEntry(void);
static void DelayPauseExit(void);
static void EndEntry(void);
static void FaultDo(void);
static void FaultEntry(void);
static void PauseDo(void);
static void PauseEntry(void);
static void PauseExit(void);
static void ProgrammingDo(void);
static void ProgrammingEntry(void);
static void Reset(void);
static void ResetTasks(void);
static void Resume(void);
static void RunDo(void);
static void RunEntry(void);
static void RunExit(void);
static void Start(void);

// -- Private Function Prototypes used by the state machine for standard state machine functions --
static void EventDelay(void);
static void EventEnd(void);
static void EventFault(void);
static void EventFaultReset(void);
static void EventPause(void);
static void EventReset(void);
static void EventResume(void);
static void EventStart(void);
static void SupervisorExecute(void);
static void SupervisorInitialize(void);
static void SupervisorTerminate(void);

// !! StateEngineCrank cannot update this code if you change the footer !!
// End of StateEngineCrank Auto-Generated Code.

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------
//! External type for service cycle/phase/start requests.
PACKED typedef struct EXTERNAL_REQUEST_STRUCT
{
    uint8 Phase;
    uint8 Step;
    uint8 Start_Value;
} EXTERNAL_REQUEST_TYPE;

//! Structure type to hold compartment supervisor data.
PACKED typedef struct SUPERVISOR_DATA_STRUCT
{
    uint16                  Cycle_Elapsed_Time;         //!< Run time for current or last cycle.
    uint16                  Time_Counter;               //!< 100 millisecond counter for cycle elapsed time and delay.
    uint8                   Fault_Phase;                //!< Fault phase for current compartment (if active).
    SUPERVISOR_STATE_TYPE   State;                      //!< State of supervisor for each compartment.
    BOOL_TYPE               Fault : 1;                  //!< Fault is active for current compartment.
    BOOL_TYPE               Pause_Request : 1;          //!< Pause request from non HMI source (e.g. monitors).
    BOOL_TYPE               Test_Cycle_Active : 1;      //!< Compartment 0 running test cycle all others off.
    BOOL_TYPE               Test_Cycle_Selected : 1;    //!< Test cycle selected waiting for start.
} SUPERVISOR_DATA_TYPE;

//! Supervisor regulations type for status/values from HMI regulations.
PACKED typedef struct SPV_REGULATIONS_DATA_STRUCT
{
    uint8 Selector_Value;
    uint8 Setting_File_Value;
    uint8 Start_Value;
    BOOL_TYPE Start_Changed : 1;
} SPV_REGULATIONS_DATA_TYPE;

//! Compartment of current running supervisor state machine.
static uint8 Current_Compartment;

//! Structure to store data from external request.
static EXTERNAL_REQUEST_TYPE External_Request[APPLIANCE_MAX_COMPARTMENTS];

//! Result from Interpreter per compartment
#pragma location = "_appliance_segment"
INTERPRETER_RESULT_TYPE Interpreter_Result[APPLIANCE_MAX_COMPARTMENTS];

//! Supervisor regulations compartment array. One entry per compartment (0 indicates compartment 1).
#pragma location = "_appliance_segment"
static SPV_REGULATIONS_DATA_TYPE Spv_Regulations[APPLIANCE_MAX_COMPARTMENTS];

//! Supervisor data compartment array. One entry per compartment (0 indicates compartment 1).
#pragma location = "_appliance_segment"
static SUPERVISOR_DATA_TYPE Supervisor_Data[APPLIANCE_MAX_COMPARTMENTS];

//! Copy of Supervisor_Data State (for Variable module) compartment array. One entry per compartment (0 indicates compartment 1).
#pragma location = "_appliance_segment"
SUPERVISOR_STATE_TYPE Supervisor_State[APPLIANCE_MAX_COMPARTMENTS];

//! Not OK to start bit declaration for programming, delay and pause states.
#define SUPERVISOR_STATE_NOT_OK_TO_START_BIT    (128U)

//! Variable to control delay time count down and provide status to the HMI in seconds.
//! Not static because it is extern to the Variables Module.
#pragma location = "_appliance_segment"
uint32 Supervisor_Delay_Time_Remaining[APPLIANCE_MAX_COMPARTMENTS];

// --- LOG MODULE SETTINGS ---------------------------
//! The name of this module for use by the Log module.
#define MODULE_NAME MODULE_SUPERVISOR

//! A list of log message IDs that are generated by this module.
typedef enum MODULE_SUPERVISOR_LOG_MESSAGE_ID_ENUM
{
    COMPARTMENT_OUT_OF_RANGE                = 1,    //!< Compartments start at zero.
    PROGRAMMING_CYCLE_NOT_FOUND             = 2,    //!< Error searching for programming cycle by label.
    MAX_INTERPRETER_STEPS                   = 3,    //!< Continuation of loads exceeds max steps.
    NOT_SUBMODE_NORMAL_SUPERVISOR           = 4,    //!< External call made while not in submode normal.
    AUTO_START_CYCLE_ENDED_RESTARTED        = 5,    //!< An auto-start compartment transitioned to end state.
    ATTEMPT_TO_PAUSE_CYCLE_NOT_ALLOWED      = 6,    //!< Attempt to pause a cycle that does not allow pause.
    NO_REGULATION_DEFINED_FOR_DELAY         = 7,    //!< Must have at least one delay regulation if delay feature is enabled.
    SELECTOR_CLEARED_BEFORE_EXTERNAL_START  = 8,    //!< Selector cleared before external start runs.
    INVALID_CYCLE_STATE_ON_COLD_RESET       = 9,    //!< Invalid cycle state, cycle canceled.
    TRACE_STATE                             = 100   //!< Used by state engine crank to log events.
} MODULE_SUPERVISOR_LOG_MESSAGE_ID_TYPE;

#ifndef TEST_CYCLE_STAYS_IN_END_UNTIL_RESET
    //! If ENABLED, Supervisor stays in END state until being reset by the HMI after running a Test Manager Cycle.
    //! If DISABLED, Supervisor automatically goes back to PROGRAMMING state after running a Test Manager Cycle.
    #define TEST_CYCLE_STAYS_IN_END_UNTIL_RESET     (DISABLED)
#endif

//! Macro to inform Supervisor if delay regulation should be checked.
#ifndef CYCLE_DELAY_FEATURE
    #define CYCLE_DELAY_FEATURE (DISABLED)
#endif

//! Macro to inform Supervisor if PROGRAMMING, DELAY and PAUSE NOT_OK_TO_START SYSTEM STATES are enabled.
#ifndef NOT_OK_TO_START_SYSTEM_STATES
    #define NOT_OK_TO_START_SYSTEM_STATES    (DISABLED)
#endif

#ifndef SPECIAL_CYCLE_CCRB_FROM_SELECTOR
    //! If ENABLED, special cycle CCRB will behave according to CCRB for SElECTOR.
    //! If DISABLED special cycle CCRB will be CYCLE_COLD_RESET_BEHAVIOR_RESUME
    #define SPECIAL_CYCLE_CCRB_FROM_SELECTOR    (DISABLED)
#endif

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
static void CheckRegulations(void);
static void ConfigureEndCycle(uint8 phase);
static void SupervisorFaultObserver(uint8 fault_monitor_id);
static void SupervisorPreFaultObserver(uint8 fault_monitor_id);
static void TestCycleEnd(void);
#if (CYCLE_DELAY_FEATURE == ENABLED)
    static void GetDelay(BOOL_TYPE check_for_change);
#endif
static BOOL_TYPE OkToStart(void);

//-------------------------------------- PUBLIC (Variables) -----------------------------------------------------------

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================
//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      It Initializes the module Supervisor and its variables
 */
void Supervisor__Initialize(void)
{
    for (uint8 index = 0; index < APPLIANCE_MAX_COMPARTMENTS; index++)
    {
        External_Request[index].Start_Value = 0;
        External_Request[index].Phase = PHASE_STEP_NONE;
        External_Request[index].Step = PHASE_STEP_NONE;
    }

    Fault__RegisterFaultEvents(SupervisorFaultObserver);
    Fault__RegisterPreFaultEvents(SupervisorPreFaultObserver);

    Interpreter__Initialize();
    if (Micro__GetResetMode() == MICRO_RESET_MODE_COLD)
    {
        if (NVData__GetLastRestoreStatus()->Bit.Appliance_Data_Recovered != NVDATA_RESTORE_OKAY)
        {
            memset(Interpreter_Result, 0x00, sizeof(Interpreter_Result));
            memset(Spv_Regulations, 0x00, sizeof(Spv_Regulations));
            memset(Supervisor_Data, 0x00, sizeof(Supervisor_Data));
            memset(Supervisor_Delay_Time_Remaining, 0x00, sizeof(Supervisor_Delay_Time_Remaining));

            for (Current_Compartment = 0; Current_Compartment < APPLIANCE_MAX_COMPARTMENTS; Current_Compartment++)
            {
                SupervisorInitialize();
            }
        }
        else
        {
            // ---------- Manage cold reset behavior for each compartment ----------
            for (Current_Compartment = 0; Current_Compartment < APPLIANCE_MAX_COMPARTMENTS; Current_Compartment++)
            {
                // Determine what cycle was running.
                CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);

                // Determine what type of cycle was running
                CYCLE_STATE_TYPE cycle_state = Selector__GetCycleState(0, Current_Compartment, cps.Cycle);

                // Determine cold reset behavior according to selected cycle.
                uint8 selected_cycle;
                switch (cycle_state)
                {
                    case CYCLE_STATE_PROGRAMMING:
                    case CYCLE_STATE_RUN:
                    case CYCLE_STATE_FAULT:
                    #if (SPECIAL_CYCLE_CCRB_FROM_SELECTOR == DISABLED)
                        case CYCLE_STATE_DELAY:
                        case CYCLE_STATE_END:
                        case CYCLE_STATE_PAUSE:
                    #endif
                        selected_cycle = cps.Cycle;
                        break;

                    #if (SPECIAL_CYCLE_CCRB_FROM_SELECTOR == ENABLED)
                        case CYCLE_STATE_DELAY:
                        case CYCLE_STATE_END:
                        case CYCLE_STATE_PAUSE:
                            selected_cycle = cps.Phase;
                            break;
                    #endif

                    default:
                        // Set CYCLE_STATE_PROGRAMMING - ccrb from Setting file will be CYCLE_COLD_RESET_BEHAVIOR_CANCEL
                        selected_cycle = CYCLE_STATE_PROGRAMMING;
                        LOG_ADD_EXCEPTION(INVALID_CYCLE_STATE_ON_COLD_RESET, Current_Compartment);
                        break;
                }
                CYCLE_COLD_RESET_BEHAVIOR_TYPE ccrb = Selector__GetColdResetBehavior(PRIMARY_SETTING_FILE, Current_Compartment, selected_cycle);

                // Manage cold reset behavior as determined by selection in setting file for the selected cycle.
                if (ccrb == CYCLE_COLD_RESET_BEHAVIOR_SELECT)
                {
                    ccrb = CYCLE_COLD_RESET_SELECT_BEHAVIOR(Current_Compartment, cycle_state);
                }
                switch (ccrb)
                {
                    default: //lint-fallthrough
                    case CYCLE_COLD_RESET_BEHAVIOR_CANCEL:
                        #if (STATISTICS_SAVED_CYCLES > 0)
                            Statistics__CycleStopped(FALSE, Current_Compartment, cps, Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time);
                        #endif
                        if (Supervisor__IsTestCycleActive() == TRUE)
                        {
                            if (Current_Compartment == 0)
                            {
                                TestCycleEnd();
                                EventReset();
                            }
                        }
                        else
                        {
                            EventReset();
                        }
                        break;

                    case CYCLE_COLD_RESET_BEHAVIOR_PAUSE:
                        if (cycle_state != CYCLE_STATE_DELAY)
                        {
                            EventPause();
                        }
                        break;

                    case CYCLE_COLD_RESET_BEHAVIOR_RESTART_PHASE:
                    {
                        CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);
                        INTERPRETER_CONFIG_TYPE interpreter_config;
                        interpreter_config.Setting_File_Index = 0;
                        interpreter_config.Cycle = cps.Cycle;
                        interpreter_config.Phase = cps.Phase;
                        interpreter_config.Statement = 0;
                        interpreter_config.Cycle_State = Selector__GetCycleState(interpreter_config.Setting_File_Index,
                                                                                 Current_Compartment, interpreter_config.Cycle);
                        Interpreter__Configure(Current_Compartment, &interpreter_config);
                    }
                        break;

                    case CYCLE_COLD_RESET_BEHAVIOR_RESTART_CYCLE:
                    {
                        CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);
                        INTERPRETER_CONFIG_TYPE interpreter_config;
                        interpreter_config.Setting_File_Index = 0;
                        interpreter_config.Cycle = cps.Cycle;
                        if ((cycle_state == CYCLE_STATE_DELAY) ||
                            (cycle_state == CYCLE_STATE_END) ||
                            (cycle_state == CYCLE_STATE_PAUSE))
                        {
                            interpreter_config.Phase = cps.Phase;
                        }
                        else
                        {
                            interpreter_config.Phase = 0;
                        }
                        interpreter_config.Statement = 0;
                        interpreter_config.Cycle_State = Selector__GetCycleState(interpreter_config.Setting_File_Index,
                                                                                 Current_Compartment, interpreter_config.Cycle);
                        Interpreter__Configure(Current_Compartment, &interpreter_config);
                    }
                        break;

                    case CYCLE_COLD_RESET_BEHAVIOR_RESUME:
                        break;
                }
            }
        }
    }

    for (Current_Compartment = 0; Current_Compartment < APPLIANCE_MAX_COMPARTMENTS; Current_Compartment++)
    {
        if (Supervisor_Data[Current_Compartment].State == STATE_SUPERVISOR_NONE)
        {
            SupervisorInitialize();
        }
        else if (Supervisor_Data[Current_Compartment].State == STATE_SUPERVISOR_FAULT)
        {
            Fault__CompartmentManaged(Current_Compartment);

            sint32 fault_id;
            if (Variable__GetValue(VARIABLE_FAULT, Current_Compartment, &fault_id) == TRUE)
            {
                if (fault_id == 0)
                {
                    EventFaultReset();
                }
            }
        }
     }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Engage test_cycle_id in compartment 0.
 *  @details    Set all other compartments to the Run State with Selector set to OFF.
 *              Set flag to indicate only compartment 0 should run.
 *
 *  @param      uint8 compartment - identifier for appliance compartment. (0 based, 0 for single compartment appliances)
 *
 *  @return     PASS_FAIL_TYPE result
 *  @retval     PASS - Valid test_cycle_id Test engaged.
 *  @retval     FAIL - Invalid test_cycle_id Test Not engaged.
 */
PASS_FAIL_TYPE Supervisor__EngageTest(uint8 test_cycle_id)
{
    PASS_FAIL_TYPE result = PASS;

    CYCLE_NAME_TYPE cycle_name = Selector__GetCycleNameId(Spv_Regulations[0].Setting_File_Value, 0, test_cycle_id);
    if ((cycle_name == CYCLE_NAME_EOL_0) || (cycle_name == CYCLE_NAME_EOL_1) || (cycle_name == CYCLE_NAME_EOL_2)||
        (cycle_name == CYCLE_NAME_SERVICE_0) || (cycle_name == CYCLE_NAME_SERVICE_1))
    {
        Fault__SetTestEngaged(TRUE);

        #if (API012EXP_FEATURE == ENABLED)
            API012Exp__SetTestCycleEngaged(TRUE);
        #endif

        Supervisor_Data[0].Test_Cycle_Selected = TRUE;

        // Reset all compartments to off.
        for (uint8 compartment = 0; compartment < APPLIANCE_MAX_COMPARTMENTS; compartment++)
        {
            // Send compartment to programming state.
            if (Supervisor_State[compartment] == STATE_SUPERVISOR_FAULT)
            {
                Supervisor__SetStart(compartment, FS_RESET_FAULT);
            }
            else
            {
                Supervisor__SetStart(compartment, FS_RESET_CONDITION);
            }

            // Run compartment Supervisor to accept reset.
            Supervisor__Run(compartment);

            // Set compartment selector to OFF
            Supervisor__SetCycle(compartment, 0);
        }

        Supervisor_Data[0].Test_Cycle_Active = TRUE;
        Supervisor_Data[0].Test_Cycle_Selected = FALSE;

        // Start service cycle.
        Supervisor__SetCycle(0, test_cycle_id);
        result = Supervisor__SetStart(0, FS_START_CONDITION);
    }
    else
    {
        result = FAIL;
    }

    if (result == FAIL)
    {
        TestCycleEnd();
        EventReset();
    }

    return (result);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Return boolean value to indicate if a test cycle is active.
 *
 *  @return     BOOL_TYPE
 *  @retval     TRUE - Test Cycle is active.
 *  @retval     FALSE - Test Cycle is not active.
 */
BOOL_TYPE Supervisor__IsTestCycleActive(void)
{
    return (Supervisor_Data[0].Test_Cycle_Active);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Return selected cycle that is in paused delay.
 *
 *  @param      uint8 compartment - identifier for appliance compartment. (0 based, 0 for single compartment appliances)
 *
 *  @return     uint8
 *  @retval     0 - Not in paused delay
 *  @retval     1-255 - Selector value for delayed cycle.
 */
uint8 Supervisor__IsInPausedDelay(uint8 compartment)
{
    if (Supervisor_Data[compartment].State == STATE_SUPERVISOR_DELAY_PAUSE)
    {
        return (Spv_Regulations[compartment].Selector_Value);
    }
    else
    {
        return(0);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Get cycle time of current or last running cycle for compartment.
 *
 *  @param      uint8 compartment - identifier for appliance compartment. (0 based, 0 for single compartment appliances)
 *
 *  @return     uint16 Cycle_Elapsed_Time - Compartment elapsed time of current or last running cycle (in seconds).
 */
uint16 Supervisor__GetCycleTime(uint8 compartment)
{
    return Supervisor_Data[compartment].Cycle_Elapsed_Time;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Run cycle/phase through Interpreter for every Supervisor State.
 *              Called by ApplianceMgr in a loop for each compartment.
 *
 *  @param      uint8 compartment - identifier for appliance compartment. (0 based, 0 for single compartment appliances)
 */
void Supervisor__Run(uint8 compartment)
{
    if (compartment >= APPLIANCE_MAX_COMPARTMENTS)
    {
        LOG_ADD_EXCEPTION(COMPARTMENT_OUT_OF_RANGE, compartment);
    }
    else if (Mode__GetSubMode() != SUBMODE_NORMAL_SUPERVISOR)
    {
        LOG_ADD_EXCEPTION(NOT_SUBMODE_NORMAL_SUPERVISOR, compartment);
    }
    else
    {
        ActivateTask__RunCompartment(compartment);

        uint8 interpreter_step_count = 0;
        do
        {
            interpreter_step_count++;
            Interpreter_Result[compartment] = Interpreter__Run(compartment);
        } while (((Interpreter_Result[compartment] == INTERPRETER_RESULT_END_STEP)||
                  ((Interpreter_Result[compartment] == INTERPRETER_RESULT_END_PHASE) &&
                   (Supervisor_Data[compartment].State == STATE_SUPERVISOR_RUN))) &&
                 (interpreter_step_count < INTERPRETER_MAX_STEPS));

        if (interpreter_step_count >= INTERPRETER_MAX_STEPS)
        {
            LOG_ADD_EXCEPTION(MAX_INTERPRETER_STEPS, interpreter_step_count);
        }

        Current_Compartment = compartment;

        if (Interpreter_Result[Current_Compartment] == INTERPRETER_RESULT_INVALID_COMPARTMENT)
        {
            SupervisorTerminate();
            Supervisor_State[Current_Compartment] = Supervisor_Data[Current_Compartment].State;
        }

        CheckRegulations();

        if (Supervisor_Data[compartment].Fault == TRUE)
        {
            Supervisor_Data[compartment].Fault = FALSE;
            EventFault();
        }

        SupervisorExecute();
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Request cycle to be run. Only allowed in programming state.
 *
 *  @param      uint8 compartment - Identifier for appliance compartment (0 based, 0 for single compartment appliances).
 *  @param      uint8 cycle - Cycle to be run.
 *
 *  @retval     PASS - Supervisor state OK. Compartment and cycle are valid.
 *  @retval     FAIL - Supervisor state not OK. Compartment or cycle are invalid.
 */
PASS_FAIL_TYPE Supervisor__SetCycle(uint8 compartment, uint8 cycle)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;
    PASS_FAIL_TYPE result;

    result = FAIL;

    if (compartment >= APPLIANCE_MAX_COMPARTMENTS)
    {
        LOG_ADD_EXCEPTION(COMPARTMENT_OUT_OF_RANGE, compartment);
    }
    else if (Mode__GetSubMode() != SUBMODE_NORMAL_SUPERVISOR)
    {
        LOG_ADD_EXCEPTION(NOT_SUBMODE_NORMAL_SUPERVISOR, compartment);
    }
    else
    {
        CYCLE_STATE_TYPE cycle_state = Selector__GetCycleState(Spv_Regulations[compartment].Setting_File_Value, compartment, cycle);
        if ((Supervisor_Data[compartment].State == STATE_SUPERVISOR_PROGRAMMING) &&
            ((cycle_state == CYCLE_STATE_RUN) ||(cycle_state == CYCLE_STATE_PROGRAMMING)))
        {
            Spv_Regulations[compartment].Selector_Value = cycle;

            interpreter_config.Setting_File_Index = Spv_Regulations[compartment].Setting_File_Value;
            interpreter_config.Cycle = 0;
            interpreter_config.Phase = Spv_Regulations[compartment].Selector_Value;
            interpreter_config.Statement = 0;
            interpreter_config.Cycle_State = Selector__GetCycleState(interpreter_config.Setting_File_Index,
                                                                     compartment, interpreter_config.Cycle);
            Interpreter__Configure(compartment, &interpreter_config);

            #if (TIME_ESTIMATION_FEATURE == ENABLED)
                Estimator__EstimateFast();
            #endif

            result = PASS;
        }
    }

    return (result);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Request phase to be run. Only allowed in STATE_RUN.
 *
 *  @param      uint8 compartment - Identifier for appliance compartment (0 based, 0 for single compartment appliances).
 *  @param      uint8 phase - Phase (0-127), PHASE_STEP_FORWARD, PHASE_STEP_BACKWARD or PHASE_STEP_NONE.
 *
 *  @retval     PASS - Supervisor state OK and compartment is valid.
 *  @retval     FAIL - Supervisor state not OK or compartment is invalid.
 */
PASS_FAIL_TYPE Supervisor__SetPhase(uint8 compartment, uint8 phase)
{
    PASS_FAIL_TYPE result;

    result = FAIL;

    if (compartment >= APPLIANCE_MAX_COMPARTMENTS)
    {
        LOG_ADD_EXCEPTION(COMPARTMENT_OUT_OF_RANGE, compartment);
    }
    else if (Mode__GetSubMode() != SUBMODE_NORMAL_SUPERVISOR)
    {
        LOG_ADD_EXCEPTION(NOT_SUBMODE_NORMAL_SUPERVISOR, compartment);
    }
    else
    {
        if (Supervisor_Data[compartment].State == STATE_SUPERVISOR_RUN)
        {
            External_Request[compartment].Phase = phase;
            result = PASS;
       }
    }

    return (result);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Request step to be run. Only allowed in STATE_RUN.
 *
 *  @param      uint8 compartment - Identifier for appliance compartment (0 based, 0 for single compartment appliances).
 *  @param      uint8 step - Step (0-127), PHASE_STEP_FORWARD, PHASE_STEP_BACKWARD or PHASE_STEP_NONE.
 *
 *  @retval     PASS - Supervisor state OK and compartment is valid.
 *  @retval     FAIL - Supervisor state not OK or compartment is invalid.
 */
PASS_FAIL_TYPE Supervisor__SetStep(uint8 compartment, uint8 step)
{
    PASS_FAIL_TYPE result;

    result = FAIL;

    if (compartment >= APPLIANCE_MAX_COMPARTMENTS)
    {
        LOG_ADD_EXCEPTION(COMPARTMENT_OUT_OF_RANGE, compartment);
    }
    else if (Mode__GetSubMode() != SUBMODE_NORMAL_SUPERVISOR)
    {
        LOG_ADD_EXCEPTION(NOT_SUBMODE_NORMAL_SUPERVISOR, compartment);
    }
    else
    {
        if (Supervisor_Data[compartment].State == STATE_SUPERVISOR_RUN)
        {
            External_Request[compartment].Step = step;
            result = PASS;
       }
    }

    return (result);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Handle start regulations from an external device.
 *
 *  @param      uint8 compartment - Identifier for appliance compartment (0 based, 0 for single compartment appliances).
 *  @param      REGULATIONS_START_COMMAND_TYPE start_value - Start command value.
 *
 *  @retval     PASS - Supervisor state OK. Start command accepted.
 *  @retval     FAIL - Supervisor state not OK. Start command rejected.
 */
PASS_FAIL_TYPE Supervisor__SetStart(uint8 compartment, REGULATIONS_START_COMMAND_TYPE start_value)
{
    PASS_FAIL_TYPE result;
    CYCLE_PHASE_STATEMENT_TYPE cps;

    result = FAIL;

    if (compartment >= APPLIANCE_MAX_COMPARTMENTS)
    {
        LOG_ADD_EXCEPTION(COMPARTMENT_OUT_OF_RANGE, compartment);
    }
    else if (Mode__GetSubMode() != SUBMODE_NORMAL_SUPERVISOR)
    {
        LOG_ADD_EXCEPTION(NOT_SUBMODE_NORMAL_SUPERVISOR, compartment);
    }
    else
    {
        cps = Interpreter__GetCurrentStatement(compartment);

        switch (start_value)
        {
            case FS_DONT_CARE:
                External_Request[compartment].Start_Value = FS_DONT_CARE;
                result = PASS;
                break;

            case FS_START_CONDITION:
                if ( (((Supervisor_Data[compartment].State == STATE_SUPERVISOR_PROGRAMMING) && (cps.Phase != 0)) ||
                    (Supervisor_Data[compartment].State == STATE_SUPERVISOR_PAUSE)) &&
                    /*Need to use macro here in order to not affect Supervisor_State */
                    (OK_TO_START_CYCLE(compartment) == TRUE))
                {
                    External_Request[compartment].Start_Value = FS_START_CONDITION;
                    result = PASS;
                }
                break;

            case FS_RESET_CONDITION:
                External_Request[compartment].Start_Value = FS_RESET_CONDITION;
                result = PASS;
                break;

            case FS_PAUSE_CONDITION:
                if ((Supervisor_Data[compartment].State == STATE_SUPERVISOR_RUN) ||
                    (Supervisor_Data[compartment].State == STATE_SUPERVISOR_DELAY))
                {
                    External_Request[compartment].Start_Value = FS_PAUSE_CONDITION;
                    result = PASS;
                }
                break;

            case FS_RESET_FAULT:
                if (Supervisor_Data[compartment].State == STATE_SUPERVISOR_FAULT)
                {
                    External_Request[compartment].Start_Value = FS_RESET_FAULT;
                    result = PASS;
                }
                break;

            case FS_RESTART_CONDITION:
                break;

            case FS_END_CONDITION:
                if (Supervisor_Data[compartment].State == STATE_SUPERVISOR_RUN)
                {
                    EventEnd();
                    result = PASS;
                }
                break;

            default:
                break;
        }
    }

    return (result);
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================
//---------------------------------------------------------------------------------------------------------------------
/**
 *   @brief      Set local start regulation based on Regulations API.
 *               Handle "FS_DONT_CARE" for all states.
 *               Handle "FS_RESET_CONDITION" for all states from the UI or external device. Generate evReset if true.
 *               Store all other start commands for state specific processing.
 */
static void CheckRegulations(void)
{
    uint8 position;
    uint8 value;
    BOOL_TYPE start_regulation_changed = FALSE;

    position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);
    if (position != REGULATION_INVALID)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = Regulations__IsRegulationChanged(position);
        if (Spv_Regulations[Current_Compartment].Start_Changed == TRUE)
        {
            start_regulation_changed = TRUE;
            value = (uint8)Regulations__GetRequestedMainValueByPosition(position);
            if (value == FS_DONT_CARE)
            {
                Spv_Regulations[Current_Compartment].Start_Changed = FALSE;
                Regulations__AcceptRequestByPosition(position);
            }
            else if (value == FS_RESET_CONDITION)
            {
                Spv_Regulations[Current_Compartment].Start_Changed = FALSE;
                Regulations__AcceptRequestByPosition(position);

                #if (STATISTICS_SAVED_CYCLES > 0)
                    CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);
                    Statistics__CycleStopped(FALSE, Current_Compartment, cps, Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time);
                #endif

                TestCycleEnd();
                EventReset();
            }
            else
            {
                Spv_Regulations[Current_Compartment].Start_Value = value;
            }
        }
    }

    #if (CYCLE_DELAY_FEATURE == ENABLED)
        GetDelay(TRUE);
    #endif

    // External request does not require regulation to be present.
    // Only check external if no current request through regulation.
    if (start_regulation_changed == FALSE)
    {
        if (External_Request[Current_Compartment].Start_Value == FS_RESET_CONDITION)
        {
            External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;

            #if (STATISTICS_SAVED_CYCLES > 0)
                CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);
                Statistics__CycleStopped(FALSE, Current_Compartment, cps, Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time);
            #endif

            TestCycleEnd();
            EventReset();
        }
    }

    // Always accept set phase/step don't care value in order to detect later changes.
    uint8 phase_position = Regulations__GetPositionByFunctionID(FUNC_ID_ADVANCE_PHASE, Current_Compartment);
    if ((phase_position != REGULATION_INVALID) && (Regulations__IsRegulationChanged(phase_position) == TRUE))
    {
        value = (uint8)Regulations__GetRequestedMainValueByPosition(phase_position);
        if (value == PHASE_STEP_NONE)
        {
            Regulations__AcceptRequestByPosition(phase_position);
        }
    }

    uint8 step_position = Regulations__GetPositionByFunctionID(FUNC_ID_ADVANCE_STEP, Current_Compartment);
    if ((step_position != REGULATION_INVALID) && (Regulations__IsRegulationChanged(step_position) == TRUE))
    {
        value = (uint8)Regulations__GetRequestedMainValueByPosition(step_position);
        if (value == PHASE_STEP_NONE)
        {
            Regulations__AcceptRequestByPosition(step_position);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Supervisor Fault Observer. Sets fault when called.
 *
 *  @param      fault_monitor_id - The fault monitor that contains all fault information.
 *                  Additional information about the fault can be found through the Fault module.
 */
static void SupervisorFaultObserver(uint8 fault_monitor_id)
{
    FAULT_CONFIG_TYPE* data;
    uint8 compartment;

    Fault__GetEventInfo(fault_monitor_id, &data);
    if ((data != NULL) && data->Phase != UCHAR_MAX)
    {
        compartment = data->Compartment;

        Supervisor_Data[compartment].Fault_Phase = data->Phase;
        Supervisor_Data[compartment].Fault = TRUE;
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Supervisor Pre-Fault Observer. Pauses cycle based on pre-fault configuration when called.
 *
 *  @param      fault_monitor_id - The fault monitor that contains all fault information.
 *                  Additional information about the pre-fault can be found through the Fault module.
 */
static void SupervisorPreFaultObserver(uint8 fault_monitor_id)
{
    FAULT_CONFIG_TYPE* data;

    Fault__GetEventInfo(fault_monitor_id, &data);

    if ((data != NULL) && data->Pause_Cycle == TRUE)
    {
        Current_Compartment = data->Compartment;
        EventPause();
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Check to see if a test cycle is running and handle if true.
 *
 */
static void TestCycleEnd(void)
{
    if (Supervisor_Data[0].Test_Cycle_Active == TRUE)
    {
        Supervisor_Data[0].Test_Cycle_Active = FALSE;

        uint8 position = Regulations__GetPositionByFunctionID(FUNC_ID_SELECTOR, 0);
        if (position != REGULATION_INVALID)
        {
            Spv_Regulations[0].Selector_Value = (uint8)Regulations__GetRequestedMainValueByPosition(position);
        }

        Fault__SetTestEngaged(FALSE);

        #if (API012EXP_FEATURE == ENABLED)
            API012Exp__SetTestCycleEngaged(FALSE);
        #endif

        // Reset other compartments.
        uint8 current_compartment_saved = Current_Compartment;
        for (uint8 compartment = 1; compartment < APPLIANCE_MAX_COMPARTMENTS; compartment++)
        {
            Current_Compartment = compartment;
            EventReset();
        }
        Current_Compartment = current_compartment_saved;
    }
}

#if (CYCLE_DELAY_FEATURE == ENABLED)
//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Get the delay time.
 *
 *  @param      BOOL_TYPE check_for_change - TRUE:  Regulation must change to read value.
 *                                           FALSE: Read value regardless of regulation change.
 */
static void GetDelay(BOOL_TYPE check_for_change)
{
    REGULATIONS_FUNCTION_ID_TYPE func_id_delay_table[3] = {FUNC_ID_DELAY_HH, FUNC_ID_DELAY_MM, FUNC_ID_DELAY_SS};
    sint32 delay_values[3] = {0,0,0};

    uint8 position;
    BOOL_TYPE delay_changed = FALSE;
    for (uint8 index = 0; index < 3; index++)
    {
        position = Regulations__GetPositionByFunctionID(func_id_delay_table[index], Current_Compartment);
        if (position != REGULATION_INVALID)
        {
            delay_values[index] = Regulations__GetRequestedMainValueByPosition(position);

            if ((check_for_change == TRUE) && (Regulations__IsRegulationChanged(position) == TRUE))
            {
                delay_changed = TRUE;
                Regulations__AcceptRequestByPosition(position);
            }
        }
    }

    if ((delay_changed == TRUE) || (check_for_change == FALSE))
    {
        Supervisor_Delay_Time_Remaining[Current_Compartment] = delay_values[0] * (sint32)3600;
        Supervisor_Delay_Time_Remaining[Current_Compartment] += delay_values[1] * (sint32)60;
        Supervisor_Delay_Time_Remaining[Current_Compartment] += delay_values[2];
    }
}
#endif  //(CYCLE_DELAY_FEATURE == ENABLED)

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Check if it is OK to start a cycle for compartment.
 *              Set/Clear SUPERVISOR_STATE_NOT_OK_TO_START_BIT in Supervisor_State[compartment] (external status).
 *
 *  @detail     Only called in Programming, Delay, Delay Pause and Pause (Do and entry functions).
 *              Only affects SUPERVISOR_STATE_NOT_OK_TO_START_BIT for those states.
 *
 *  @retval TRUE - It is OK to start a cycle.
 *  @retval FALSE - It is not OK to start a cycle.
 *
 */
static BOOL_TYPE OkToStart(void)
{
    BOOL_TYPE ok_to_start = FALSE;

#if (NOT_OK_TO_START_SYSTEM_STATES == ENABLED)
    /* Use project specific macro to determine if it is ok to start */
    if (OK_TO_START_CYCLE(Current_Compartment) == TRUE)
    {
        ok_to_start = TRUE;
        Supervisor_State[Current_Compartment] = (SUPERVISOR_STATE_TYPE)(Supervisor_Data[Current_Compartment].State &
                                                                        ~SUPERVISOR_STATE_NOT_OK_TO_START_BIT);
    }
    else
    {
        Supervisor_State[Current_Compartment] = (SUPERVISOR_STATE_TYPE)(Supervisor_Data[Current_Compartment].State |
                                                                        SUPERVISOR_STATE_NOT_OK_TO_START_BIT);
    }
#else
    ok_to_start = OK_TO_START_CYCLE(Current_Compartment);
    Supervisor_State[Current_Compartment] = Supervisor_Data[Current_Compartment].State;
#endif

    return (ok_to_start);
}

//=====================================================================================================================
//  --- Private State Machine Customization Functions -----------------------------------------------------------------
//=====================================================================================================================
//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void DelayDo(void)
{
    BOOL_TYPE start_accepted = FALSE;
    BOOL_TYPE pause_accepted = FALSE;

    BOOL_TYPE ok_to_start = OkToStart();

    // Decrement delay time remaining first to insure we don't get extra 100 millisecond before start when reaching 0.
    if (Supervisor_Data[Current_Compartment].Time_Counter >= 9)
    {
        Supervisor_Data[Current_Compartment].Time_Counter = 0;
        if (Supervisor_Delay_Time_Remaining[Current_Compartment] != 0)
        {
            Supervisor_Delay_Time_Remaining[Current_Compartment]--;
        }
    }
    else
    {
        Supervisor_Data[Current_Compartment].Time_Counter++;
    }

    //Check for restart or pause from HMI.
    if (Spv_Regulations[Current_Compartment].Start_Changed == TRUE)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = FALSE;

        uint8 position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);

        if (Spv_Regulations[Current_Compartment].Start_Value == FS_RESTART_CONDITION)
        {
            start_accepted = TRUE;
            Regulations__AcceptRequestByPosition(position);
        }
        else if (Spv_Regulations[Current_Compartment].Start_Value == FS_PAUSE_CONDITION)
        {
            pause_accepted = TRUE;
            Regulations__AcceptRequestByPosition(position);
        }
        else
        {
            Regulations__RejectRequestByPosition(position);
        }
    }

    // Check if ready to start else if pause is requested.
    if (((Supervisor_Delay_Time_Remaining[Current_Compartment] == 0) || (start_accepted == TRUE)) && (ok_to_start == TRUE))
    {
        INTERPRETER_CONFIG_TYPE interpreter_config;
        interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
        interpreter_config.Cycle = Spv_Regulations[Current_Compartment].Selector_Value;
        interpreter_config.Phase = 0;
        interpreter_config.Statement = 0;
        interpreter_config.Cycle_State = Selector__GetCycleState(interpreter_config.Setting_File_Index,
                                                                 Current_Compartment, interpreter_config.Cycle);
        Interpreter__Configure(Current_Compartment, &interpreter_config);

        Supervisor_Delay_Time_Remaining[Current_Compartment] = 0;
        EventStart();
    }
    else if ((External_Request[Current_Compartment].Start_Value == FS_PAUSE_CONDITION) || (pause_accepted))
    {
        External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;
        EventPause();
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void DelayEntry(void)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;

    Supervisor_Data[Current_Compartment].Time_Counter = 0;

    interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
    interpreter_config.Cycle = Selector__GetStateCycle(Spv_Regulations[Current_Compartment].Setting_File_Value,
                                                         Current_Compartment, CYCLE_STATE_DELAY);
    interpreter_config.Phase = Spv_Regulations[Current_Compartment].Selector_Value;
    interpreter_config.Statement = 0;
    interpreter_config.Cycle_State = CYCLE_STATE_DELAY;
    Interpreter__Configure(Current_Compartment, &interpreter_config);

    // Set Supervisor_State (external status) and set/clear SUPERVISOR_STATE_NOT_OK_TO_START_BIT accordingly.
    OkToStart();

    ActivateTask__ResetCompartment(Current_Compartment);

    //Keep DELAY_ENTRY as last item in Entry function
    //Insure that entry tasks have been completed before notifying application of State Change.
    DELAY_ENTRY(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void DelayPauseDo(void)
{
    uint8 start_position;

    BOOL_TYPE ok_to_start = OkToStart();

    if (Spv_Regulations[Current_Compartment].Start_Changed)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = FALSE;

        if (Spv_Regulations[Current_Compartment].Start_Value == FS_START_CONDITION)
        {
            start_position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);

            if (ok_to_start == TRUE)
            {
                Regulations__AcceptRequestByPosition(start_position);

                EventResume();
            }
            else
            {
                Regulations__RejectRequestByPosition(start_position);
            }
        }
    }
    else if (External_Request[Current_Compartment].Start_Value == FS_START_CONDITION)
    {
        External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;
        if (ok_to_start == TRUE)
        {
            EventResume();
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void DelayPauseEntry(void)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;

    interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
    interpreter_config.Cycle = Selector__GetStateCycle(Spv_Regulations[Current_Compartment].Setting_File_Value,
                                                         Current_Compartment, CYCLE_STATE_PAUSE);
    interpreter_config.Phase = Spv_Regulations[Current_Compartment].Selector_Value;
    interpreter_config.Statement = 0;
    interpreter_config.Cycle_State = CYCLE_STATE_PAUSE;
    Interpreter__Configure(Current_Compartment, &interpreter_config);

    // Set Supervisor_State (external status) and set/clear SUPERVISOR_STATE_NOT_OK_TO_START_BIT accordingly.
    OkToStart();

    //Keep PAUSE_DELAY_ENTRY as last item in Entry function
    //Insure that entry tasks have been completed before notifying application of State Change.
    DELAY_PAUSE_ENTRY(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void DelayPauseExit(void)
{
    //Keep DELAY_PAUSE_EXIT as last item in Exit function
    //Insure that exit tasks have been completed before notifying application of State Change.
    DELAY_PAUSE_EXIT(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void EndEntry(void)
{
    // Round cycle time results
    Supervisor_Data[Current_Compartment].Time_Counter++;
    if (Supervisor_Data[Current_Compartment].Time_Counter >= 5)
    {
        Supervisor_Data[Current_Compartment].Time_Counter = 0;
        if (Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time < 0xFFFF)
        {
            Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time++;
        }
    }

    #if (STATISTICS_SAVED_CYCLES > 0)
        CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);
        Statistics__CycleStopped(TRUE, Current_Compartment, cps, Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time);
    #endif

    Supervisor_Data[Current_Compartment].Time_Counter = 0;
    Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time = 0;

    if (AUTO_START(Current_Compartment))
    {
        TestCycleEnd();
        EventReset();
        LOG_ADD_EXCEPTION(AUTO_START_CYCLE_ENDED_RESTARTED, Current_Compartment);
    }
    else if (Supervisor_Data[0].Test_Cycle_Active == TRUE)
    {
        #if (TEST_CYCLE_STAYS_IN_END_UNTIL_RESET == ENABLED)
            // Save selected test cycle for running the test cycle END macro if ENABLED.
            uint8 test_cycle = Spv_Regulations[Current_Compartment].Selector_Value;
        #endif
        TestCycleEnd();     // Ends test and sets Selector_Value from test cycle back to regulation value

        #if (TEST_CYCLE_STAYS_IN_END_UNTIL_RESET == DISABLED)
            EventReset();
        #else
            ConfigureEndCycle(test_cycle);
        #endif
    }
    else
    {
        ConfigureEndCycle(Spv_Regulations[Current_Compartment].Selector_Value);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * Configure the interpreter to run the END Cycle (Macro).
 */
static void ConfigureEndCycle(uint8 phase)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;

    interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
    interpreter_config.Cycle = Selector__GetStateCycle(Spv_Regulations[Current_Compartment].Setting_File_Value,
                                                         Current_Compartment, CYCLE_STATE_END);
    interpreter_config.Phase = phase;
    interpreter_config.Statement = 0;
    interpreter_config.Cycle_State = CYCLE_STATE_END;
    Interpreter__Configure(Current_Compartment, &interpreter_config);

    Interpreter__ClearSubCycleStatus(Current_Compartment);

    Supervisor_State[Current_Compartment] = Supervisor_Data[Current_Compartment].State;

    ActivateTask__ResetCompartment(Current_Compartment);

    //Keep END_ENTRY as last item in Entry function (Don't call if EventReset called above).
    //Insure that entry tasks have been completed before notifying application of State Change.
    END_ENTRY(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void FaultDo(void)
{
    uint8 start_position;

    if (Spv_Regulations[Current_Compartment].Start_Changed == TRUE)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = FALSE;

        start_position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);

        if (Spv_Regulations[Current_Compartment].Start_Value == FS_RESET_FAULT)
        {
            if (Fault__IsOkToExitFaultState(Current_Compartment) == TRUE)
            {
                Regulations__AcceptRequestByPosition(start_position);
                TestCycleEnd();
                EventFaultReset();
            }
            else
            {
                Regulations__RejectRequestByPosition(start_position);
            }
        }
        else
        {
            Regulations__RejectRequestByPosition(start_position);
        }
    }
    else if (External_Request[Current_Compartment].Start_Value == FS_RESET_FAULT)
    {
        External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;
        if (Fault__IsOkToExitFaultState(Current_Compartment) == TRUE)
        {
            TestCycleEnd();
            EventFaultReset();
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void FaultEntry(void)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;

    // Statistics responds to the error event when it happens, Supervisor waits for it's next run of the state machine.
    // OK to clear because Statistics has already recorded the elapsed cycle time.
    Supervisor_Data[Current_Compartment].Time_Counter = 0;
    Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time = 0;

    interpreter_config.Setting_File_Index = PRIMARY_SETTING_FILE;
    interpreter_config.Cycle = Selector__GetStateCycle(Spv_Regulations[Current_Compartment].Setting_File_Value,
                                                         Current_Compartment, CYCLE_STATE_FAULT);
    interpreter_config.Phase = Supervisor_Data[Current_Compartment].Fault_Phase;
    interpreter_config.Statement = 0;
    interpreter_config.Cycle_State = CYCLE_STATE_FAULT;

    Interpreter__Configure(Current_Compartment, &interpreter_config);

    Interpreter__ClearSubCycleStatus(Current_Compartment);

    Supervisor_State[Current_Compartment] = Supervisor_Data[Current_Compartment].State;

    ActivateTask__ResetCompartment(Current_Compartment);

    Fault__CompartmentManaged(Current_Compartment);

    //Keep FAULT_ENTRY as last item in Entry function
    //Insure that entry tasks have been completed before notifying application of State Change.
    FAULT_ENTRY(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void PauseDo(void)
{
    uint8 start_position;

    BOOL_TYPE ok_to_start = OkToStart();

    if (Spv_Regulations[Current_Compartment].Start_Changed)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = FALSE;

        if (Spv_Regulations[Current_Compartment].Start_Value == FS_START_CONDITION)
        {
            start_position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);

            if (ok_to_start == TRUE)
            {
                Regulations__AcceptRequestByPosition(start_position);

                Interpreter__RestoreFromPauseResumeData(Current_Compartment);

                Interpreter__Resume(Current_Compartment);

                EventResume();
            }
            else
            {
                Regulations__RejectRequestByPosition(start_position);
            }
        }
    }
    else if (External_Request[Current_Compartment].Start_Value == FS_START_CONDITION)
    {
        External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;
        if (ok_to_start == TRUE)
        {
            Interpreter__RestoreFromPauseResumeData(Current_Compartment);
            Interpreter__Resume(Current_Compartment);
            EventResume();
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void PauseEntry(void)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;

    Interpreter__SavePauseResumeData(Current_Compartment);

    interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
    interpreter_config.Cycle = Selector__GetStateCycle(Spv_Regulations[Current_Compartment].Setting_File_Value,
                                                         Current_Compartment, CYCLE_STATE_PAUSE);
    interpreter_config.Phase = Interpreter__GetCurrentStatement(Current_Compartment).Cycle;
    interpreter_config.Statement = 0;
    interpreter_config.Cycle_State = CYCLE_STATE_PAUSE;
    Interpreter__Configure(Current_Compartment, &interpreter_config);

    // Set Supervisor_State (external status) and set/clear SUPERVISOR_STATE_NOT_OK_TO_START_BIT accordingly.
    OkToStart();

    ActivateTask__PauseCompartment(Current_Compartment);
    Maintain__PauseEntry(Current_Compartment);

    //Keep PAUSE_ENTRY as last item in Entry function
    //Insure that entry tasks have been completed before notifying application of State Change.
    PAUSE_ENTRY(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void PauseExit(void)
{
    //Keep PAUSE_EXIT as last item in Exit function
    //Insure that exit tasks have been completed before notifying application of State Change.
    PAUSE_EXIT(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void ProgrammingDo(void)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;
    uint8 position;
    BOOL_TYPE start_accepted;
    uint8 value;

    BOOL_TYPE ok_to_start = OkToStart();

    start_accepted = FALSE;
    position = Regulations__GetPositionByFunctionID(FUNC_ID_SELECTOR, Current_Compartment);
    if (position != REGULATION_INVALID)
    {
        if (Regulations__IsRegulationChanged(position) == TRUE)
        {
            value = (uint8)Regulations__GetRequestedMainValueByPosition(position);
            CYCLE_STATE_TYPE state = Selector__GetCycleState(Spv_Regulations[Current_Compartment].Setting_File_Value, Current_Compartment, value);
            if ((state == CYCLE_STATE_RUN) || (state == CYCLE_STATE_PROGRAMMING))
            {
                if (value == 0) // OFF
                {
                    ActivateTask__ResetCompartment(Current_Compartment);
                }

                Spv_Regulations[Current_Compartment].Selector_Value = value;
                Regulations__AcceptRequestByPosition(position);

                interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
                interpreter_config.Cycle = 0;
                interpreter_config.Phase = Spv_Regulations[Current_Compartment].Selector_Value;
                interpreter_config.Statement = 0;
                interpreter_config.Cycle_State = Selector__GetCycleState(interpreter_config.Setting_File_Index, Current_Compartment, interpreter_config.Cycle);;
                Interpreter__Configure(Current_Compartment, &interpreter_config);

                #if (TIME_ESTIMATION_FEATURE == ENABLED)
                    Estimator__EstimateFast();
                #endif
            }
            else
            {
                Regulations__RejectRequestByPosition(position);
            }
        }
    }

    if (Spv_Regulations[Current_Compartment].Start_Changed == TRUE)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = FALSE;

        if (Spv_Regulations[Current_Compartment].Start_Value == FS_START_CONDITION)
        {
            position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);

            if ((ok_to_start == TRUE) && (Spv_Regulations[Current_Compartment].Selector_Value != 0))
            {
                start_accepted = TRUE;
                Regulations__AcceptRequestByPosition(position);
            }
            else
            {
                Regulations__RejectRequestByPosition(position);
            }
        }
    }

    if ((External_Request[Current_Compartment].Start_Value == FS_START_CONDITION) || (start_accepted == TRUE))
    {
        External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;

        // Selector_Value can change in communication thread after external request.
        // Make sure selector value is still valid before starting.
        if (Spv_Regulations[Current_Compartment].Selector_Value != 0)
        {
            if (Supervisor_Delay_Time_Remaining[Current_Compartment] == 0)
            {
                interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
                interpreter_config.Cycle = Spv_Regulations[Current_Compartment].Selector_Value;
                interpreter_config.Phase = 0;
                interpreter_config.Statement = 0;
                interpreter_config.Cycle_State = Selector__GetCycleState(interpreter_config.Setting_File_Index,
                                                                         Current_Compartment, interpreter_config.Cycle);
                Interpreter__Configure(Current_Compartment, &interpreter_config);

                EventStart();
            }
            else
            {
                EventDelay();
            }
        }
        else
        {
            LOG_ADD_EXCEPTION(SELECTOR_CLEARED_BEFORE_EXTERNAL_START,
                              COMBINE_BYTES(Current_Compartment,Spv_Regulations[Current_Compartment].Selector_Value));
            TestCycleEnd();
            EventReset();
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void ProgrammingEntry(void)
{
    //! Structure to initialize interpreter for refrigerator.
    INTERPRETER_CONFIG_TYPE interpreter_config;

    Supervisor_Data[Current_Compartment].Time_Counter = 0;
    Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time = 0;

#if (CYCLE_DELAY_FEATURE == ENABLED)
    GetDelay(FALSE);
#endif

    Fault__CompartmentReset(Current_Compartment);

    // Set Supervisor_State (external status) and set/clear SUPERVISOR_STATE_NOT_OK_TO_START_BIT accordingly.
    BOOL_TYPE ok_to_start = OkToStart();

    if ((Supervisor_Data[0].Test_Cycle_Selected == FALSE) &&
        (AUTO_START(Current_Compartment) == TRUE) &&
        (ok_to_start == TRUE))
    {
        if (Selector__GetCycleState(Spv_Regulations[Current_Compartment].Setting_File_Value, Current_Compartment, 1) == CYCLE_STATE_RUN)
        {
            interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
            interpreter_config.Cycle = 1;
            Spv_Regulations[Current_Compartment].Selector_Value = 1;
            interpreter_config.Phase = 0;
            interpreter_config.Statement = 0;
            interpreter_config.Cycle_State = CYCLE_STATE_RUN;
            Interpreter__Configure(Current_Compartment, &interpreter_config);
            EventStart();
        }
        Supervisor_State[Current_Compartment] = Supervisor_Data[Current_Compartment].State;

        ActivateTask__ResetCompartment(Current_Compartment);
    }
    else
    {
        //Keep PROGRAMMING_ENTRY as last item in Entry function.
        //Insure that entry tasks have been completed before notifying application of State Change.
        PROGRAMMING_ENTRY(Current_Compartment);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void RunDo(void)
{
    uint8 sf_index;
    uint8 phase_position;
    uint8 start_position;
    uint8 step_position;
    INTERPRETER_STATUS_TYPE status;
    uint8 value;

    Supervisor_Data[Current_Compartment].Time_Counter++;
    if (Supervisor_Data[Current_Compartment].Time_Counter >= 10)
    {
        Supervisor_Data[Current_Compartment].Time_Counter = 0;
        if (Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time < 0xFFFF)
        {
            Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time++;
        }
    }

    phase_position = Regulations__GetPositionByFunctionID(FUNC_ID_ADVANCE_PHASE, Current_Compartment);
    step_position = Regulations__GetPositionByFunctionID(FUNC_ID_ADVANCE_STEP, Current_Compartment);

    // If else structure that handles requests according to priority.
    // Highest priorities are cancel/pause requests from Interpreter.
    // Next are regulation changes from HMI.
    // After that are external requests from API005 or other modules.
    // Lastly if no requests to change cycle execution, run the estimator count down.
    if (Interpreter_Result[Current_Compartment] == INTERPRETER_RESULT_END_CYCLE)
    {
        EventEnd();
    }
    else if (Interpreter_Result[Current_Compartment] == INTERPRETER_RESULT_END_CYCLE_RESET)
    {
        #if (STATISTICS_SAVED_CYCLES > 0)
            CYCLE_PHASE_STATEMENT_TYPE cps = Interpreter__GetCurrentStatement(Current_Compartment);
            Statistics__CycleStopped(FALSE, Current_Compartment, cps, Supervisor_Data[Current_Compartment].Cycle_Elapsed_Time);
        #endif

        TestCycleEnd();
        EventReset();
    }
    else if (Interpreter_Result[Current_Compartment] == INTERPRETER_RESULT_PAUSE)
    {
        status = Interpreter__GetStatus();
        sf_index = Interpreter__GetCurrentSFIndex(Current_Compartment);
        if (Selector__OkToPauseCycle(sf_index, Current_Compartment, status.Main_Cycle[Current_Compartment]) == TRUE)
        {
            EventPause();
        }
        else
        {
            LOG_ADD_EXCEPTION(ATTEMPT_TO_PAUSE_CYCLE_NOT_ALLOWED, Current_Compartment);
        }
    }
    else if (Spv_Regulations[Current_Compartment].Start_Changed == TRUE)
    {
        Spv_Regulations[Current_Compartment].Start_Changed = FALSE;

        start_position = Regulations__GetPositionByFunctionID(FUNC_ID_START, Current_Compartment);
        if (Spv_Regulations[Current_Compartment].Start_Value == FS_PAUSE_CONDITION)
        {
            status = Interpreter__GetStatus();
            sf_index = Interpreter__GetCurrentSFIndex(Current_Compartment);
            if (Selector__OkToPauseCycle(sf_index, Current_Compartment, status.Main_Cycle[Current_Compartment]) == TRUE)
            {
                Regulations__AcceptRequestByPosition(start_position);
                EventPause();
            }
            else
            {
                Regulations__RejectRequestByPosition(start_position);
            }
        }
        else
        {
            // FS_RESET_CONDITION and FS_END_CONDITION are handled globally regardless of Supervisor State.
            // All other start values are invalid for the RUN state.
            Regulations__RejectRequestByPosition(start_position);
        }
    }
    else if ((phase_position != REGULATION_INVALID) && (Regulations__IsRegulationChanged(phase_position) == TRUE))
    {
        value = (uint8)Regulations__GetRequestedMainValueByPosition(phase_position);
        PHASE_STEP_RESULT_TYPE phase_result = Interpreter__SetPhase(Current_Compartment, value);
        if (phase_result == PHASE_STEP_RESULT_PASS)
        {
            Regulations__AcceptRequestByPosition(phase_position);
        }
        else if (phase_result == PHASE_STEP_RESULT_END)
        {
            Regulations__AcceptRequestByPosition(step_position);
            EventEnd();
        }
        else
        {
            Regulations__RejectRequestByPosition(phase_position);
        }
    }
    else if ((step_position != REGULATION_INVALID) && (Regulations__IsRegulationChanged(step_position) == TRUE))
    {
        value = (uint8)Regulations__GetRequestedMainValueByPosition(step_position);
        PHASE_STEP_RESULT_TYPE step_result = Interpreter__SetStep(Current_Compartment, value);
        if (step_result == PHASE_STEP_RESULT_PASS)
        {
            Regulations__AcceptRequestByPosition(step_position);
        }
        else if (step_result == PHASE_STEP_RESULT_END)
        {
            Regulations__AcceptRequestByPosition(step_position);
            EventEnd();
        }
        else
        {
            Regulations__RejectRequestByPosition(step_position);
        }
    }
    else if ((External_Request[Current_Compartment].Start_Value == FS_PAUSE_CONDITION) ||
             (Supervisor_Data[Current_Compartment].Pause_Request == TRUE))
    {
        External_Request[Current_Compartment].Start_Value = FS_DONT_CARE;
        Supervisor_Data[Current_Compartment].Pause_Request = FALSE;

        status = Interpreter__GetStatus();
        sf_index = Interpreter__GetCurrentSFIndex(Current_Compartment);
        if (Selector__OkToPauseCycle(sf_index, Current_Compartment, status.Main_Cycle[Current_Compartment]) == TRUE)
        {
            EventPause();
        }
        else
        {
            LOG_ADD_EXCEPTION(ATTEMPT_TO_PAUSE_CYCLE_NOT_ALLOWED, Current_Compartment);
        }
    }
    else if (External_Request[Current_Compartment].Phase != PHASE_STEP_NONE)
    {
        if (Interpreter__SetPhase(Current_Compartment, External_Request[Current_Compartment].Phase) == PHASE_STEP_RESULT_END)
        {
            EventEnd();
        }

        External_Request[Current_Compartment].Phase = PHASE_STEP_NONE;
    }
    else if (External_Request[Current_Compartment].Step != PHASE_STEP_NONE)
    {
        if (Interpreter__SetStep(Current_Compartment, External_Request[Current_Compartment].Step) == PHASE_STEP_RESULT_END)
        {
            EventEnd();
        }
        External_Request[Current_Compartment].Step = PHASE_STEP_NONE;
    }
    else
    {
        #if (TIME_ESTIMATION_FEATURE == ENABLED)
            Estimator__Countdown(Current_Compartment);
        #endif
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void RunEntry(void)
{
    Supervisor_State[Current_Compartment] = Supervisor_Data[Current_Compartment].State;

    //Keep RUN_ENTRY as last item in Entry function
    //Insure that entry tasks have been completed before notifying application of State Change.
    RUN_ENTRY(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void RunExit(void)
{
    // Application specific macro. Informs application of exit from RUN state.
    RUN_EXIT(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void Reset(void)
{
    INTERPRETER_CONFIG_TYPE interpreter_config;

    interpreter_config.Setting_File_Index = Spv_Regulations[Current_Compartment].Setting_File_Value;
    interpreter_config.Cycle = 0;
    interpreter_config.Phase = Spv_Regulations[Current_Compartment].Selector_Value;
    interpreter_config.Statement = 0;
    interpreter_config.Cycle_State = CYCLE_STATE_PROGRAMMING;
    Interpreter__Configure(Current_Compartment, &interpreter_config);

    Interpreter__ClearSubCycleStatus(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void Resume(void)
{
    ActivateTask__ResumeCompartment(Current_Compartment);

    RESUME(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void Start(void)
{
    ActivateTask__ResetCompartment(Current_Compartment);

    #if (STATISTICS_SAVED_CYCLES > 0)
        Statistics__CycleStarted(Current_Compartment, Spv_Regulations[Current_Compartment].Selector_Value);
    #endif


    START(Current_Compartment);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 */
static void ResetTasks(void)
{
    ActivateTask__ResetCompartment(Current_Compartment);
}

//=====================================================================================================================
//=== StateEngineCrank (v2.8) Auto-Generated Functions for Supervisor State Machine
//=====================================================================================================================
// !! StateEngineCrank cannot update this code if you change the header !!


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Delay Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventDelay(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_PROGRAMMING:
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_DELAY;
            DelayEntry();                           // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the End Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventEnd(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_RUN:
            RunExit();                              // Exit function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_END_OF_CYCLE;
            EndEntry();                             // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Fault Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventFault(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_PROGRAMMING:
        case STATE_SUPERVISOR_DELAY:
        case STATE_SUPERVISOR_END_OF_CYCLE:
        case STATE_SUPERVISOR_FAULT:
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_FAULT;
            FaultEntry();                           // Entry Function.
            break;

        case STATE_SUPERVISOR_RUN:
            RunExit();                              // Exit function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_FAULT;
            FaultEntry();                           // Entry Function.
            break;

        case STATE_SUPERVISOR_PAUSE:
            PauseExit();                            // Exit function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_FAULT;
            FaultEntry();                           // Entry Function.
            break;

        case STATE_SUPERVISOR_DELAY_PAUSE:
            DelayPauseExit();                       // Exit function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_FAULT;
            FaultEntry();                           // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Fault Reset Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventFaultReset(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_FAULT:
            Reset();                                // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PROGRAMMING;
            ProgrammingEntry();                     // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Pause Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventPause(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_RUN:
            RunExit();                              // Exit function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PAUSE;
            PauseEntry();                           // Entry Function.
            break;

        case STATE_SUPERVISOR_DELAY:
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_DELAY_PAUSE;
            DelayPauseEntry();                      // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Reset Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventReset(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_PROGRAMMING:
        case STATE_SUPERVISOR_DELAY:
        case STATE_SUPERVISOR_END_OF_CYCLE:
            Reset();                                // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PROGRAMMING;
            ProgrammingEntry();                     // Entry Function.
            break;

        case STATE_SUPERVISOR_RUN:
            RunExit();                              // Exit function.
            Reset();                                // Transition action function.
            ResetTasks();                           // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PROGRAMMING;
            ProgrammingEntry();                     // Entry Function.
            break;

        case STATE_SUPERVISOR_PAUSE:
            PauseExit();                            // Exit function.
            Reset();                                // Transition action function.
            ResetTasks();                           // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PROGRAMMING;
            ProgrammingEntry();                     // Entry Function.
            break;

        case STATE_SUPERVISOR_DELAY_PAUSE:
            DelayPauseExit();                       // Exit function.
            Reset();                                // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PROGRAMMING;
            ProgrammingEntry();                     // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Resume Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventResume(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_PAUSE:
            PauseExit();                            // Exit function.
            Resume();                               // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_RUN;
            RunEntry();                             // Entry Function.
            break;

        case STATE_SUPERVISOR_DELAY_PAUSE:
            DelayPauseExit();                       // Exit function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_DELAY;
            DelayEntry();                           // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * Process the Start Event for the Supervisor state machine.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void EventStart(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_PROGRAMMING:
        case STATE_SUPERVISOR_DELAY:
            Start();                                // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_RUN;
            RunEntry();                             // Entry Function.
            break;

        case STATE_SUPERVISOR_DELAY_PAUSE:
            DelayPauseExit();                       // Exit function.
            Start();                                // Transition action function.
            Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_RUN;
            RunEntry();                             // Entry Function.
            break;

        default:
            // Event is ignored for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * The main execution function for the Supervisor state machine.
 * This function should be called periodically while the state machine is running.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void SupervisorExecute(void)
{
    switch(Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_PROGRAMMING:
            ProgrammingDo();                        // Do function.
            break;

        case STATE_SUPERVISOR_RUN:
            RunDo();                                // Do function.
            break;

        case STATE_SUPERVISOR_DELAY:
            DelayDo();                              // Do function.
            break;

        case STATE_SUPERVISOR_PAUSE:
            PauseDo();                              // Do function.
            break;

        case STATE_SUPERVISOR_FAULT:
            FaultDo();                              // Do function.
            break;

        case STATE_SUPERVISOR_DELAY_PAUSE:
            DelayPauseDo();                         // Do function.
            break;

        default:
            // There is nothing to do for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * The initialization function for the Supervisor state machine.
 * This function resets the state machine to the initial state and executes the initial state's
 * entry function.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void SupervisorInitialize(void)
{
    Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_PROGRAMMING;
    ProgrammingEntry();                             // Entry Function.
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * The termination function for the Supervisor state machine.
 * This function disables the state machine. Calls to StateMachineExecute() will have no effect.
 * The StateMachineInitialize() function must be called again to restart the state machine.
 * If the current state has an exit function, it will be called.
 *
 *              >>> DO NOT MODIFY THIS FUNCTION <<<
 *
 * This function was auto-generated and must not be edited manually.
 * Change the UML used to generate this function and regenerate the source code.
 */
static void SupervisorTerminate(void)
{
    switch (Supervisor_Data[Current_Compartment].State)
    {
        case STATE_SUPERVISOR_RUN:
            RunExit();                              // Exit function.
            break;

        case STATE_SUPERVISOR_PAUSE:
            PauseExit();                            // Exit function.
            break;

        case STATE_SUPERVISOR_DELAY_PAUSE:
            DelayPauseExit();                       // Exit function.
            break;

        default:
            // There is nothing to do for all other values of Supervisor_Data[Current_Compartment].State.
            break;
    }
    Supervisor_Data[Current_Compartment].State = STATE_SUPERVISOR_NONE;
}


// !! StateEngineCrank cannot update this code if you change the footer !!
// End of StateEngineCrank Auto-Generated Code.
