/**
*
*
*/

#ifndef MC_TYPES_H
#define MC_TYPES_H


/** \defgroup mc_def_data_types Motor control; defined data types
* @{
*/

#if defined(MOTOR_NUMBER_TYPE_FLOATING)
typedef mc_float_t mcmp_t;         ///< type for motor physicals
typedef mc_float_t mcsp_t;         ///< motor speed type
typedef mc_float_t mcvc_t;         ///< generic type for voltage and current
typedef mc_float_t mcc_t;          ///< motor current type
typedef mc_float_t mcv_t;          ///< motor voltage type
typedef mc_float_t mcr_t;          ///< motor winding resistance
typedef mc_float_t mcdeg_t;        ///< motor position type
typedef mc_float_t mcrad_t;        ///< motor radial position type
typedef mc_float_t mctri_t;        ///< trigonemetric result type
typedef mc_float_t mcpow_t;        ///< power type
typedef mc_float_t sec_t;          ///< time type - unit : second
typedef mc_float_t msec_t;         ///< time type - unit : milisecond
typedef mc_uint32_t htck_t;        ///< handler execution timer
typedef mc_float_t rmpc_t;         ///< ramp current
typedef mc_float_t rmpv_t;         ///< ramp voltage
typedef mc_float_t rmps_t;         ///< ramp speed
typedef mc_float_t duty_t;         ///< data types for pwm duty cycles
typedef mc_int32_t mindx_t;        ///< data type for motor index
typedef mc_uint32_t mccnt_t;       ///< data types for counters
typedef mc_float_t err_t;          ///< error data type for pi controllers
typedef mc_uint16_t mcstc_t;       ///< state command to decide motor control state flow
typedef mc_uint8_t sector_t;       ///< motor drive sector type
typedef mc_float_t timing_t;       ///< timing calcultion type
typedef mc_int8_t  sign_t;         ///< sign of a value
typedef mc_int32_t step_t;         ///< step type for process flow
typedef mc_bool_t  mcps_t;         ///< state to show if parameters are loadded
typedef mc_uint32_t freq_t;        ///< frequency value for motor control handlers
typedef sec_t   period_t;          ///< period value for motor control handlers
typedef mc_float_t mcw_t;          ///< radial frequency type    
typedef mc_float_t mcfc_t;         ///< filter coefficient type
typedef mc_float_t mctemp_t;       ///< temperature type

typedef mc_float_t ratio_t;        ///< type for ratio variables
#elif defined(MOTOR_NUMBER_TYPE_FRACTIONAL)

#else
#error "Motor control layer number type should be defined!"
#endif

typedef struct {
    contPiKp_t kp;
    contPiKi_t ki;
    mcvc_t antiWindupCoef;
    mcvc_t antiWindupLimit;
} mcpPiController_t;    ///< type for pi controller parameters

typedef struct {
    mcvc_t u;
    mcvc_t v;
    mcvc_t w;
} mcuvw_t;     ///< type for 3-phase generic variables

typedef struct {
    mcvc_t a;
    mcvc_t b;
} mcab_t;      ///< type for alpha-beta generic variables

typedef struct {
    mcvc_t d;
    mcvc_t q;
} mcdq_t;      ///< type for dq generic variables

typedef struct{
    duty_t u;
    duty_t v;
    duty_t w;
} duvw_t;       ///< type for phase duty cycles

typedef struct{
    sign_t u;
    sign_t v;
    sign_t w;
} suvw_t;    ///< type for sign of phase values

typedef struct {
    mcdeg_t th;
    mctri_t cos;
    mctri_t sin;
} th_t;    ///< actual motor position datas

typedef unsigned char mcst_t;
#define JUST_POWERED    ((mcst_t)0x00U)
#define INIT            ((mcst_t)0x01U)
#define STOPPED         ((mcst_t)0x02U)
#define STARTUP         ((mcst_t)0x03U)
#define RAMP            ((mcst_t)0x04U)
#define TURNOFF         ((mcst_t)0x05U)
#define FAULT           ((mcst_t)0x06U)
#define BOOTSTRAP       ((mcst_t)0x07U)
#define ON_THE_FLY	    ((mcst_t)0x08U)
#define RESET_DRIVE     ((mcst_t)0x09U)

typedef unsigned char bsstep_t; /* bootstrap step type */
#define BOOTSTRAP_NOT_APPLIED   ((bsstep_t)0x00U)
#define BOOTSTRAP_INITIATED     ((bsstep_t)0x01U)
#define BOOTSTRAP_FOR_PHASE_U   ((bsstep_t)0x02U)
#define BOOTSTRAP_FOR_PHASE_V   ((bsstep_t)0x03U)
#define BOOTSTRAP_FOR_PHASE_W   ((bsstep_t)0x04U)
#define BOOTSTRAP_APPLIED       ((bsstep_t)0x05U)   

typedef unsigned char octech_t; /* open loop to closed loop transition method */
#define DIRECT_TRANSITION               ((octech_t)0x00U)
#define PI_CONTROLLED_TRANSITION        ((octech_t)0x01U)
#define CALCULATED_TORQUE_TRANSITION    ((octech_t)0x02U)

typedef unsigned char startup_ol_state_t;   /* open loop startup states */
#define STARTUP_OL_STATE_PASSIVE        ((startup_ol_state_t)0x00U)
#define STARTUP_OL_STATE_TH_ALIGN_0     ((startup_ol_state_t)0x07U)
#define STARTUP_OL_STATE_TH_ALIGN_1ST  ((startup_ol_state_t)0x01U)
#define STARTUP_OL_STATE_TH_ALIGN_2ND  ((startup_ol_state_t)0x02U)
#define STARTUP_OL_STATE_MEC_ALIGN_1ST ((startup_ol_state_t)0x03U)
#define STARTUP_OL_STATE_MEC_ALIGN_2ND ((startup_ol_state_t)0x04U)
#define STARTUP_OL_STATE_MEC_ALIGN_3RD ((startup_ol_state_t)0x08U)
#define STARTUP_OL_STATE_RAMPUP         ((startup_ol_state_t)0x05U)
#define STARTUP_OL_STATE_TRANSITION     ((startup_ol_state_t)0x06U)

typedef unsigned char turnoff_ol_state_t;   /* open loop turnoff states */
#define TURNOFF_OL_STATE_PASSIVE        ((turnoff_ol_state_t)0x00U)
#define TURNOFF_OL_ROTATING             ((turnoff_ol_state_t)0x01U)
#define TURNOFF_OL_CURRENT_STATE1       ((turnoff_ol_state_t)0x02U)
#define TURNOFF_OL_CURRENT_STATE2       ((turnoff_ol_state_t)0x03U)
#define TURNOFF_OL_CURRENT_STATE3       ((turnoff_ol_state_t)0x04U)
#define TURNOFF_OL_PWM_STATE_SELECTION  ((turnoff_ol_state_t)0x05U)

typedef unsigned char lmethod_t;  /* control loop method */
#define METHOD_OPEN_LOOP    ((lmethod_t)0x00U)
#define METHOD_CLOSED_LOOP  ((lmethod_t)0x01U)
#define METHOD_DISABLE_PWM  ((lmethod_t)0x02U)

typedef unsigned char mcstate_t;
#define NOTREADY    ((mcstate_t)0x00U)
#define READY       ((mcstate_t)0x01U)

typedef signed char mcrt_t;   /* motor rotation type */
#define ROT_NO      ((mcrt_t)0)
#define ROT_CW      ((mcrt_t)1)
#define ROT_CCW     ((mcrt_t)-1)

typedef signed char mcacc_t;
#define ACC_NO      ((mcacc_t)0)
#define ACC_POS     ((mcacc_t)1)
#define ACC_NEG     ((mcacc_t)-1)

typedef unsigned char rampController_t;     /* Ramp controller selection type */
#define RAMP_CONTROLLER_NOT_SELECTED    ((rampController_t)0x00U)
#define RAMP_CONTROLLER_RPIOBSERVER     ((rampController_t)0x01U)
#define RAMP_CONTROLLER_EKFOBSERVER     ((rampController_t)0x02U)

typedef unsigned char tmode_t;
#define TRANSITION_MODE_NOT_SELECTED    ((tmode_t)0x00U)
#define TRANSITION_MODE_DIRECT          ((tmode_t)0x01U)
#define TRANSITION_MODE_ANGLE_SWITCHING ((tmode_t)0x02U)

typedef unsigned char mc_hwHb_state_t;
#define MC_HW_HB_STATE_PASSIVE  ((mc_hwHb_state_t)0)
#define MC_HW_HB_STATE_ACTIVE   ((mc_hwHb_state_t)1)

typedef unsigned char mcSpControllerRegion_t;
#define MC_SP_CONTROLLER_REGION_0       ((mcSpControllerRegion_t)0x00)
#define MC_SP_CONTROLLER_REGION_1       ((mcSpControllerRegion_t)0x01)
#define MC_SP_CONTROLLER_REGION_2       ((mcSpControllerRegion_t)0x02)
#define MC_SP_CONTROLLER_REGION_3       ((mcSpControllerRegion_t)0x03)

typedef unsigned char mcDecouplingMethod_t;
#define MC_DECOUPLING_METHOD_NCALC      ((mcDecouplingMethod_t)0X00)    // decoupling is not applied
#define MC_DECOUPLING_METHOD_CCALC      ((mcDecouplingMethod_t)0X01)    // current based calculation
#define MC_DECOUPLING_METHOD_VCALC      ((mcDecouplingMethod_t)0X02)    // voltage based calculation

typedef signed char mc_control_state_t;     // shows motor control state for the related motor
#define MC_CONTROL_STATE_ACTIVE     1       // means motor speed request is permitted
#define MC_CONTROL_STATE_DISABLED   0       // means requested to disable motor control requests
#define MC_CONTROL_STATE_PASSIVE    -1      // means motor control speed request is passive

typedef unsigned char mcDtCompType_t;   // deadtime compensation type according to driver structure
#define DTCOMP_NONE                 0   // no compenastion is needed in the motor control layer
#define DTCOMP_BIDIRECTIONAL        1   // driver type that needs to be compensated for both; negative and positive current values
#define DTCOMP_UNIDIRECTIONAL_NEG   2   // driver type that needs to be compensated only negative current values
#define DTCOMP_UNIDIRECTIONAL_POS   3   // driver type that needs to be compensated only positive current values

typedef struct {
    mcdq_t idq_thalign_0;       ///< thermal align dq current for 0 stage - no thermal process, just alignment
    mcdq_t idq_thalign_1st;     ///< thermal align dq current for 1st stage
    mcdq_t idq_thalign_2nd;     ///< thermal align dq current for 2nd stage
    mcdeg_t thref_thalign;      ///< thermal align reference position
    msec_t thalign_duration;    ///< thermal align current appliance duration
    msec_t thermalRefCurrentLatency;    ///< thermal align latency for reference current to stabilize

    mcdeg_t thref_mecalign;             ///< mechanic align position (use if it is not zero)
    mcdq_t idq_mecalign1st;             ///< 1st mechanic align dq current
    mcdq_t idqramp_mecalign1st;         ///< 1st mechanic align dq current ramp
    msec_t mecalign1st_duration;        ///< 1st mechanic align current appliance duration
    mcdq_t idq_mecalign2nd;             ///< 2nd mechanic align dq current
    mcdq_t idqramp_mecalign2nd;         ///< 2nd mechanic align dq current ramp
    msec_t mecalign2nd_duration;        ///< 2nd mechanic align current appliance duration
    mcdq_t idq_mecalign3rd;				///< 3rd mechanic align dq current
    mcdq_t idqramp_mecalign3rd;			///< 3rd mechanic align dq current ramp
    msec_t mecalign3rd_duration;		///< 3rd mechanic align current appliance duration

    rmpc_t idramp;      ///< id ramp
    rmpc_t iqramp;      ///< id ramp
    mcdq_t idqmin;     ///< minimum dq current
    mcdq_t idqmax;     ///< maximum dq current
    mcsp_t sprefmax;    ///< maximum refence speed
    rmps_t spreframp;   ///< reference speed ramp
    mcsp_t spgapmax;       ///< speed gap between estimated speed and reference speed
    mcsp_t minValidSp;  ///< minimum speed for valid transition
    htck_t tValidSp;    ///< time to be sure that observer calculates valid speed data

    msec_t tmax;        ///< maximum permitted elapsed time
} mcolp_t;   ///< open loop parameters

typedef struct {
    startup_ol_state_t state;    ///< open loop state
    startup_ol_state_t statePrev;    ///< open loop previous state
    htck_t timer;       ///< elapsed time in a specific state
    mcdq_t iqdref;         ///< dq current of turning vector
    mcsp_t spref;           ///< actual speed of turning vector
    htck_t tValidSpCnt;     ///< valid speed counter
} mcolv_t;   ///< open loop variables

typedef struct{
    mcolv_t ol;         ///< open loop startup variables
} mcsupv_t;     ///< startup variables

typedef struct{
    lmethod_t method;    ///< startup method
    mcolp_t ol;         ///< open loop startup parameters
} mcsupp_t;     ///< startup parameters

typedef struct{
    tmode_t mode;       ///< transition mode
    sec_t duration;     ///< transition duration time (s)
    mcdeg_t thErrMin;   ///< minimum angle error between reference and estimated position for valid transition
    mcdeg_t thErrMax;   ///< maximum angle error between reference and estimated position for valid transition
} mctransp_t;           ///< open to closed loop transition parameters

typedef struct{
    mcc_t ramp;         ///< transition ramp (Amper/SL_handler)
} mctransv_t;           ///< open to closed loop transition variables

typedef struct{
    turnoff_ol_state_t state;              ///< open loop state
    turnoff_ol_state_t statePrev;          ///< previous handler open loop state
    htck_t timer;               ///< elapsed time in a specific state
} mcturnoff_olv_t;       ///< turn-off state open loop variables

typedef struct{
    mcdq_t idq_rot;     ///< open loop rotation current for maximum t_rot milisecond or up to zero reference speed
    mcdq_t idq1;        ///< first state open loop current applied for t1 milisecond
    mcdq_t idq2;        ///< second state open loop current applied for t2 milisecond
    mcdq_t idq3;        ///< applied final open loop current after second state duration
    rmps_t spreframp;   ///< rotation ramp
    msec_t t_rot;       ///< maximum permitted duration for open loop turnoff rotation
    msec_t t1;          ///< first state open loop current appliance duration
    msec_t t2;          ///< second state open loop current appliance duration
    msec_t t3;          ///< third state open loop current appliance duration
    mc_bool_t dispwm;              ///< decide if pwm is disabled after second state open loop current
} mcturnoff_olp_t;       ///< turn-off state open loop parameters

typedef struct{
    lmethod_t method;           ///< turn off method; selecton of open loop or closed loop       
    mcturnoff_olp_t ol;         ///< open loop turn-off parameters
    msec_t disPwmLatency;	///< latentcy for motor stop after zero speed request if disable PWM mode is selected
} mcturnoffp_t;    ///< turn off state parameters

typedef struct{
    mcturnoff_olv_t ol;         ///< open loop turn-off parameters
} mcturnoffv_t;                 ///< turn off variables

typedef struct {
    mcv_t vmaxcoeff;    ///< coefficient to calculate minimum voltage for field weakening to be triggered
    mcv_t vamp_lpfcoeff;        ///< low pass filter coefficient for voltage vector amplitude
    mcv_t vmarg_lpfcoeff;       ///< low pass filter coefficient for voltage vector margine
    mcsp_t spmin;       ///< minimum speed to activate field weakening calculation
    mcsp_t sp_reset;    ///< reset variables less than reset speed
    mcpPiController_t pi;
} mcfwp_t;  ///< field weakening parameters

typedef struct {
    contPiController_t pi;
    mcv_t vmax;
    mcv_t vamp_lpfcoeff;        ///< low pass filter coefficient for voltage vector amplitude
    mcv_t vamp_filtered;        ///< filtered voltage vector amplitude
    mcv_t vmarg;
    mcv_t vmarg_lpfcoeff;       ///< low pass filter coefficient for voltage vector margine
    mcv_t vmarg_filtered;
    mcc_t idref;
} mcfwv_t;  ///< field weakening variables

typedef struct{
    mc_bool_t enabled;
    mcsp_t minsp;
    mcsp_t maxsp;
    mcc_t lpfCoeff;
} mcpMtpa_t;

typedef struct{
    mcc_t idref;
} mcvMtpa_t;

typedef struct {
    rmps_t rampmaxAcc;
    rmps_t rampminAcc;
    rmps_t rampmaxDec;
    rmps_t rampminDec;
} mcRmpCntParams_t;     ///< ramp controller parameters 

typedef struct {
    mc_uint8_t pp;      ///< number of motor poles pair
    mcr_t r;              ///< Resistance of the each winding      
    mcmp_t Ld;            ///< Ld of the motor         
    mcmp_t Lq;            ///< Lq of the motor
    mcmp_t Ke;            ///< "Electrical voltage" to "rotation(radial)" coefficient of the motor
    mcmp_t Phi;		  ///< Magnet Flux Linkage
    mcmp_t Kt;			///< Torque Constant (Nm/A)
} mcMotorPhysParams_t;          ///< motor physical parameters

typedef struct {
    bsstep_t step;      ///< variable for bootstrap process step 
    mccnt_t timer;      ///< timer for setting bootstrap timing
} mcbstrpv_t;           ///< type for bootstrap variables

typedef struct {
   mccnt_t duration;    ///< bootstrap step duration
} mcbstrpp_t;           ///< type for bootstrap parameters

typedef struct{
    mcv_t egamma, edelta;
    th_t thest;
    mcdq_t idqref;
    mcdq_t idqest;
    mcdq_t vdq;
    mcsp_t spest;
    
    contPiController_t egammaPiControllerVarId;
    contPiController_t egammaPiControllerVarIq;
    contPiController_t edeltaPiControllerVarId;
    contPiController_t edeltaPiControllerVarIq;

    period_t tsamp;
    mcv_t lpfcoeff;                     ///< low pass filter coefficient

    mcmp_t invld;
    mcmp_t invlq;    

    mcc_t iderr, iqerr;
    mccnt_t loop_cntr;                  ///< handler counter for robust pi speed observer
} mcv_rbemf_t;      ///> robust bemf pi observer data struct type

typedef struct{
    mcpPiController_t egammaId;         ///< egamma controller belongs to id
    mcpPiController_t egammaIq;         ///< egamma controller belongs to iq
    mcpPiController_t edeltaId;         ///< edelta controller belongs to id
    mcpPiController_t edeltaIq;         ///< edelta controller belongs to iq

    mcv_t lpfcoeff;                     ///< low pass filter coefficient
    mcv_t egammaMax;                    ///< permitted maximum egamma
    mcv_t edeltaMax;                    ///< permitted maximum edelta
    mcv_t edeltaMin;                    ///< permitted minimum edelta
    
    mcsp_t minsp;
} mcp_rbemf_t;            ///< parameters for robust bemf observer

typedef struct{
    mc_float_t I_EKF[4][4];
    mc_float_t Jh[2][4];
    mc_float_t Jh_trans[4][2];
    mc_float_t xk_est[4][1];
    mc_float_t Pk_inno_old[4][4];
    mc_float_t Rs;
    mc_float_t L0;
    mc_float_t Vku_old[2][1];
    mc_float_t xk_inno_old[4][1];
    mc_float_t Ts;
    mc_float_t Jf[4][4];
    mc_float_t Jf_trans[4][4];
    mc_float_t zk[2][1];
    mc_float_t hxk[2][1];
    mc_float_t multi_Jf_Pkinnoold[4][4];
    mc_float_t multi_Jf_Pkinnoold_Jftrans[4][4];
    mc_float_t Pk_est[4][4];
    mc_float_t multi_Pkest_Jhtrans[4][2];
    mc_float_t multi_Jh_Pkest[2][4];
    mc_float_t multi_Jh_Pkest_Jhtrans[2][2];
    mc_float_t sum_Jh_Pkest_Jhtrans_Rk[2][2];
    mc_float_t result_inv[2][2];
    mc_float_t Kk[4][2];
    mc_float_t inv_sum_Jh_Pkest_Jhtrans_Rk[2][2];
    mc_float_t Qk_real[4][4];
    mc_float_t Rk_real[2][2];
    mc_float_t abstract_zk_hxk[2][1];
    mc_float_t multi_Kk_abstract_zk_hxk[4][1];
    mc_float_t xk_inno[4][1];
    mc_float_t multi_Kk_Jh[4][4];
    mc_float_t abstract_I_multi_Kk_Jh[4][4];
    mc_float_t Pk_inno[4][4];
    mc_float_t xk_inno_filt;
    mc_float_t xk_inno_old_filt;
    mc_float_t theta_delta_EKF;
    mc_float_t theta_sum_EKF;
    mc_float_t speed_cnt_EKF;
    mc_float_t delta_theta_ave_EKF;
    mc_float_t speed_EKF;
    mc_float_t prev_speed_EKF;
    mc_float_t theta_EKF_deg;
    mc_float_t theta_EKF_Shift_real;
    mc_float_t prev_Theta;
    mc_uint8_t initFlagEKF;
    mc_float_t speed_EKF_Filt;
    mc_float_t invDet;

} mcv_ekf_t;

typedef struct{
    mc_float_t Qk_ol[4][4];
    mc_float_t Qk_cls[4][4];
    mc_float_t Rk[2][2];
    mc_float_t theta_EKF_Shift;
    mc_float_t speed_corr_EKF;
    mc_uint32_t sampling_speed_EKF;
    mc_uint32_t fcutSpeed;
    mc_uint32_t fcutTheta;
} mcp_ekf_t;

typedef struct{
    mcrad_t in;
    mcrad_t in_prev;
    mcw_t w;
    mcw_t wf;
    mcrad_t integ;
    mcrad_t thrad;
    sec_t dt;
    mcrad_t thrad_lpfcoeff;             ///< coefficient to apply low pass filter to radial position output
    mcrad_t thradfiltered;
    mcv_t k1;                           ///< k1 coefficient
    mcv_t k1_coeff;
    mcv_t k1_unmerged;
    mcv_t k1_merge_coeff;
    mcv_t smc;                          ///< low pass filter coefficient for a smooth response
} mcv_ato_t;

typedef struct{
    mcv_t k2;                           ///< k2 coefficient
    mcv_t smc;                          ///< low pass filter coefficient for a smooth response
    mcv_t k1min;                        ///< minimum k1 value for minimum closed loop speed
    mcv_t k1max;                        ///< maximum k1 value for maximum closed loop speed
    mcv_t k1_merge_step;                ///< merge step for deciding k1 mergecoefficient 
    mcrad_t thrad_lpfcoeff;             ///< coefficient to apply low pass filter to radial position output
} mcp_ato_t;              ///< parameters for angle tracking observer

typedef struct{
    mc_hwHb_state_t state;                    ///< state of operation
} mcv_hwHardBrake_t;            ///< hardware hard brake operation variables

typedef struct{
    mcstate_t enabled;                  ///< shows if hardware hard brake is enabled for the related motor drive operation 
    mcsp_t minsp;                       ///< minimum speed for hardware hard brake operation; release point for operation
    mcsp_t maxsp;                       ///< maximum speed for hardware hard brake operation; enterance point for operation
} mcp_hwHardBrake_t;            ///< hardware hard brake operation parameters

typedef enum{
    FS_THERMAL_STATE_PASSIVE = 0,
    FS_THERMAL_STATE_RCALC_1ST_WAIT,
    FS_THERMAL_STATE_RCALC_1ST_ACTIVE,
    FS_THERMAL_STATE_RCALC_1ST_COMPLETED,
    FS_THERMAL_STATE_RCALC_2ND_WAIT,
    FS_THERMAL_STATE_RCALC_2ND_ACTIVE,
    FS_THERMAL_STATE_RCALC_2ND_COMPLETED,
    FS_THERMAL_STATE_RCALC_COMPLETED,
    FS_THERMAL_STATE_REAL_TIME_CALC_WAIT,
    FS_THERMAL_STATE_REAL_TIME_CALC
} fs_thermal_state_t;                   ///< thermal calculation state to aware functional safety layer

typedef struct{
    fs_thermal_state_t thermal_state;           ///< thermal calculation state to aware functional safety layer    
} fs_state_t;           ///< variable set to aware functional safety layer about motor control states

typedef struct{
    mc_bool_t   spController;           ///< bypass state of the speed controller
} mc_bypass_t;

typedef struct{
    mcstate_t state;    ///< state flag to command manipulation process
    mcdq_t vdq;         ///< voltage values to overwrite actual ones; current controller outputs
    th_t vdq_tri;
} mcManipulateCc_t;     ///< if desired, variable struct is used to manipulate current controller output

typedef enum{
    MC_MODULATION_TYPE_NONE,        ///< no modulation
    MC_MODULATION_TYPE_SVPMW,       ///< standard space vector modulation
    MC_MODULATION_TYPE_SVPMW_U0,    ///< space vector modulation with 2 sectors high side transistors fully off
    MC_MODULATION_TYPE_SVPMW_U7,    ///< space vector modulation with 2 sectors low side transistors fully off
    MC_MODULATION_TYPE_DPMW,        ///< space vector modulation with 1 sector low side transistor fully off, 1 sector high side transistor off
} mc_modulation_type_t;

typedef struct{
    mcsp_t speedR1;                 ///< minimum speed for region 1
    mc_modulation_type_t typeR0;    ///< modulation type for speed region 0
    mc_modulation_type_t typeR1;    ///< modulation type for speed region 1
    duty_t deadtime;                ///< deadtime phase PWMs
    mcDtCompType_t dtCompType;      ///< deadtime compensation type according to driver structure
    duty_t  dt_trans_duty;          ///< dead time switch transition duty 
} mc_modulation_t;

typedef struct{
    mcstate_t enabled;              ///< parameter to enable breaking flying motor
    mcab_t vabFirst;                ///< Initial voltage injection value
    mcab_t vabRamp;                 ///< ramp value for voltage injection
    mcab_t vabMax;                  ///< maximum voltage injection value
    mcc_t iAveIntegPerSecMin;       ///< minimum value to inject voltage. finish process below decided value
} mcpOnTheFlyStartup_t;

typedef struct{
    mcstate_t active;               ///< variable that shows if on the fly algorithm is active
    FiltMedian_t sMed;              ///< median struct for current value
    FiltAve_t sAve;                 ///< average struct for current value
    mcc_t iMed;                     ///< calculated median filtered current
    mcc_t iAve;                     ///< calculated average median filtered current
    mcc_t iAvePrev;                 ///< previous calculated average current
    mcc_t iAveInteg;                ///< integral of calculated average current
    mcc_t iAveIntegPerSec;          ///< integral of calculated average current for a second
    mccnt_t integCntr;              ///< integral caunter for time decision
    mcab_t vabInject;               ///< instant injected voltage
} mcvOnTheFlyStartup_t;

typedef struct{
    mc_bool_t enabled;              ///< decides if torque compensation feature is enabled
    mc_float_t amp;                 ///< torque compensation amplitude as Q current
    mc_float_t pos_begin;           ///< Position that torque compensation starts to be generated
    mc_float_t pos_mid;             ///< position that frequency of the towque compensation signal changes
    mc_float_t pos_end;             ///< position that torque compensation signal generaton is stopped
} mcpTorqComp_t;

typedef struct{
    mc_float_t mPos;                ///< current motor position
    mc_float_t mPosPrev;            ///< previous motor position
    mc_float_t mecPos;              ///< calculated mechanic position
    int mPosOverflowCntr;           ///< overflow counter for motor position
    int intervalIndx;               ///< current index for the buffer that has calculated motor rotation intervals
    int intervalIndx_temp;          ///< index that is used for "for" case
    mc_uint32_t cntrFL; 
    mc_uint32_t cntrFL_prev;
    mc_uint32_t cntrFL_delta;
    mc_uint32_t counterInterval;
    mc_float_t cntrIntervalBuffer[3];
    mc_float_t cntrIntervalMin;
    mc_float_t freqCoeffFirst, freqCoeffSecond, sin, wt, torq;
    mc_float_t compCoeff, compIq;
} mcvTorqComp_t;

/** @}*/

/** \addtogroup mc_def_data_types Motor control; defined data types additional
* @{
*/

typedef struct{
    mcMotorPhysParams_t phys;   ///< motor physical parameters
    
    freq_t freqFL;              ///< Fast loop frequency for the specified motor
    freq_t freqSL;              ///< Slow loop frequency for the specified motor
    
    mcsp_t sprefmax;            ///< motor speed

    mcsp_t minsp_o2c;           ///< minimum speed for open to close loop transition 
    mcsp_t minsp_c2o;			///< minimum speed for close to open loop transition 

    mcdq_t idqrefmax;           ///< maximum reference dq current

    mcdq_t vdqmax;        ///< permitted maximum dq voltages

    mcRmpCntParams_t rampParams;        ///< parameter set for ramp controller
    mcpPiController_t piSpeedR1;        ///< parameter set for speed pi controller for speed region 1
    mcpPiController_t piSpeedR2;        ///< parameter set for speed pi controller for speed region 2
    mcpPiController_t piSpeedR3;        ///< parameter set for speed pi controller for speed region 3
    mcsp_t piSpeedR1toR2;               ///< transition speed value from pi speed region 1 to 2
    mcsp_t piSpeedR2toR3;               ///< transition speed value from pi speed region 2 to 3
    mcpPiController_t piDCurrent;       ///< parameter set for current pi controller
    period_t zcCoeffDCurrent;           ///< zero cancellation coefficient value for d current controller
    mcpPiController_t piQCurrent;       ///< parameter set for current pi controller
    period_t zcCoeffQCurrent;           ///< zero cancellation coefficient value for q current controller
    mcDecouplingMethod_t decouplingMethod;      ///< decoupling method
    mcdq_t decoupling_lpfcoeff;         ///< low pass filter coefficients for current controller decoupling process
    mcdq_t inductanceCorrectionCoeff;   ///< inductance value correction for decoupling component calculation
    
    mc_modulation_t modulation;         ///< modulation settings
    
    mcsupp_t startup;                   ///< startup parameters
    mctransp_t transition;              ///< open to closed loop transition parameters
    mc_bool_t disPwmWhenStop;		///< select the behaviour when zero speed is requested in close loop; 
    					///< if true, directs to turnoff.
					///< used for diabling PWM; use with settings in turnoff
    mcturnoffp_t turnoff;                ///< turnoff parameters
    mcdq_t stopped_current;             ///< stopped atate current

    mcfwp_t fwp;                ///< field weakening structed parameter set
    mcpMtpa_t mtpa;             ///< mtpa parameters

    mcbstrpp_t btstrp;           ///< bootstrap parameters

    ratio_t bratio;             ///< belt ratio of the mechanic system

    mccnt_t just_powered_min_duration; ///< delay duration after power up

    rampController_t rampController;    ///< selected ramp controller 
    mcp_rbemf_t rbemf;                  ///< robust bemf observer parameters
    mcp_ekf_t ekf;     			///< extended kalman filter observer parameters
    mcp_ato_t ato;                      ///< angle tracking observer parameters
    mcdeg_t thest_offset;                  ///< offset value to compensate estimated position

    mcp_hwHardBrake_t hwHardBrake;      ///< hardware hard brake operation parameters
    mcpOnTheFlyStartup_t mcOnTheFlyStartup;  ///< parameters for on the fly startup process
    mcpTorqComp_t torqComp;                 ///< parameters for torque compensation parameters
} mcCp_t;       ///< Control parameters for a motor

typedef struct{
    period_t periodFL;  ///< Fast loop period for the specified motor
    period_t periodSL;  ///< Slow loop period for the specified motor
    
    mcmp_t diffLdLq;      ///< Difference between LD and Lq

    mcps_t pstate;      ///< parameters are loaded or not
    mcstate_t calready;  ///< motor control current calibration state; ready or not
    
    mcv_t vbus;         ///< DC bara voltage
    mcv_t vbus_filtered;///< filtered DC bara voltage
    
    mcst_t state;       ///< motor control state
    mcstc_t statecom;   ///< state command

    mc_bool_t transready;  ///< ready flag for open to closed loop transition
    
    mcsp_t sprefmec;    ///< mecanic target speed per rpm; effected by belt_ratio
    mcsp_t sprefin;     ///< motor target speed per rpm
    mcsp_t sprefin_abs; ///< motor target speed per rpm
    mcsp_t sprefin_prev;///< previous motor target speed per rpm
    mcsp_t spref;       ///< motor speed reference per rpm
    mcsp_t spref_abs;   ///< motor speed reference per rpm
    mcsp_t spest;       ///< motor estimated speed per rpm
    mcsp_t spenc;       ///< motor estimated speed per rpm
    mcsp_t spact;       ///< motor actual speed per rpm
    mcsp_t spact_abs;   ///< motor actual speed per rpm
    mcsp_t spactmec;    ///< motor actual mechanic speed per rpm
    mcsp_t sperr_abs;   ///< speed error between actual motor speed and reference motor speed per rpm
    mcsp_t sperr;       ///< speed error between actual motor speed and reference motor speed per rpm
    mcsp_t spref_radps; ///< radial reference speed; radian/s
    mcsp_t spest_radps; ///< motor estimated speed; radian/s
    mcsp_t spact_radps; ///< radial actual speed; radian/s

    mcrt_t rot;         ///< rotation of the spining motor
    mcacc_t acc;        ///< shows acceleration or deceleration
    
    rmps_t rampin;      ///< requested ramp
    rmps_t ramp;        ///< applied ramp
    sec_t ramp_duration;///< requested ramp duration as second

    th_t thref;         ///< referance motor position
    th_t thenc;         ///< postition from encoder
    th_t thest;         ///< estimated motor position
    th_t thact;         ///< actual motor position
    mcdeg_t therr;      ///< position error between appied and estimated positions
    mcdeg_t th_iref_vec;    ///< position of the resultant current vector

    mcdq_t idqref;     ///< reference dq current
    mcuvw_t iuvw;      ///< current feedback for phase A-B-C
    mcuvw_t iuvw_prev; ///< current feedback for phase A-B-C in the previous current cycle
    mcab_t iab;        ///< alpha-beta current feedback
    mcdq_t idq;        ///< dq current feedback
    mcc_t iamp;        ///< amplitude of the current feedback vector
    mcc_t iamp2;       ///< square of the current feedback vector amplitude
    mcc_t iref_amp;    ///< amplitude of reference current  
    mcc_t iref_amp2;   ///< square of the amplitude of reference current  
    suvw_t siuvw;      ///< sign of the phase currents  

    mcc_t iqref_spcontroller;   ///< reference current calculated by speed pi controller  

    mcdq_t vdq;        ///< dq voltage applied
    mcab_t vab;        ///< alpha-beta voltage applied
    mcab_t vabComp;    ///< Compansated alpha-beta voltages; after DC bus ripple eliminated
    mcc_t vamp;         ///< amplitude of the applied voltage vector
    
    mcuvw_t vbemf;     ///< bemf voltage feedback for each phase

    sector_t sector;    ///< variable that shows current motor drive sector
    
    duvw_t duvw;        ///< calculated duty cycles for each phase; output of svpwm
    duvw_t duvw_comp;   ///< compensated duty cycles for each phase; output of dead time compensation
    
    mc_bool_t pwm_enabled; ///< shows if pwm is enabled
    duty_t *pwm_duty[3];    ///< addresses for operating system created pwm duty objects for motor control
    mcc_t* phcurrents[3];   ///< addresses for operating system created phase current objects for motor control

    contPiController_t piSpeed;         ///< variable set for pi speed controller
    mcSpControllerRegion_t spControllerRegion;  ///< current speed controller region
    mc_bool_t piSpeedOverwrite;         ///< over write state for speed controller parameters 
    
    mcc_t error_filtered_id;
    mcc_t error_filtered_iq;
    contPiController_t piDCurrent;      ///< parameter set for D current pi controller
    mcv_t zsDCurrentRef;                ///< latest reference D current on the zero cancellation output
    contPiController_t piQCurrent;      ///< parameter set for Q current pi controller
    mcv_t zsQCurrentRef;                ///< latest reference Q current on the zero cancellation output
    mcdq_t decoupling_lpfcoeff;         ///< low pass filter coefficients for current controller decoupling process
    mcdq_t inductanceCorrectionCoeff;   ///< inductance value correction for decoupling component calculation
    mcdq_t vdq_phase;                   ///< reference voltage vector applied to motor phases
    mcdq_t vdq_unsaturated;             ///< unsaturated reference voltage vector applied to motor
    mcdq_t vdq_decoupler_dq;            ///< decoupler voltage only effected by dq currents for current controller
    mcdq_t vdq_decoupler;               ///< decoupler voltage effected by all components for current controller
    mcManipulateCc_t cc_manipulate;     ///< if desired, manupulates current controller 

    mcpow_t apppwr;     ///< consumed apparent power for motor drive
    mcpow_t actpwr;     ///< consumed active power for motor drive

    mc_bool_t SpCntrlState;        ///< speed controller state
    mcdq_t constidq;           ///< reference dq currents if speed controller is passive;
                                ///< so constant current is applied
    mc_bool_t CurrentCntrlState;   ///< current controller state
    mcdq_t constvdq;           ///< reference dq voltage if current controller is passive; 
                                ///< so constant voltage is applied
    mcsupv_t startup;              ///< startup variables; includes open loop or closed loop startup
    mctransv_t transition;      ///< open to closed loop transition variables
    mcturnoffv_t turnoff;               ///< turnoff variables

    mccnt_t fl2slCnt;           ///< fast loop to slow loop counter
    mccnt_t flInSlCnt;           ///< fast loop counter when slow loop handling is active
    mccnt_t flInSlCntMax;           ///< fast loop maximum counter value when slow loop handling is active

    mc_bool_t loopActiveFL;        ///< one when fast loop handling is active
    mc_bool_t loopActiveSL;        ///< one when slow loop handling is active 

    mcfwv_t fwv;                ///< field weakening structed variable
    mcvMtpa_t mtpa;             ///< mtpa variables

    mcbstrpv_t btstrp;           ///< bootstrap variables

    mccnt_t just_powered_delay; ///< delay counter after power up

    mcv_rbemf_t rbemf;       ///< robust bemf pi observer defined struct object
    mcv_ekf_t ekf; 	     ///< extended kalman filter observer defined struct object
    mcv_ato_t ato;                  ///< angle tracking observer variables

    fs_state_t fs_state;        ///< motor control states to aware functional safety layer

    mcv_hwHardBrake_t hwHardBrake;      ///< hardware hard brake operation variables

    mc_bypass_t bypass;         ///< current bypass state
    mcvOnTheFlyStartup_t mcOnTheFlyStartup;  ///< variables for on the fly startup process
    mcvTorqComp_t torqComp;                 ///< parameters for torque compensation variables
} mcCv_t;       ///< Control variables for a motor

typedef struct{
    void (*mcDrvTurnOnInrushRelay)(void);
} mcFunc_t;     ///< functions used for all motor control module

typedef struct{
    void (*mcBrdPwmEnable)(void);
    void (*mcBrdPwmDisable)(void);

    void (*mcBrdPwmEnablePhaseU)(void);
    void (*mcBrdPwmEnablePhaseV)(void);
    void (*mcBrdPwmEnablePhaseW)(void);
    void (*mcBrdPwmEnablePhaseUVW)(void);
    
    void (*mcBrdMapAdcChannels)(sector_t sec);

    void (*mcBrdSampleCurrentsUVW)(sector_t sec);
    mcc_t (*mcBrdGetPhaseCurrentU)(void);
    mcc_t (*mcBrdGetPhaseCurrentV)(void);
    mcc_t (*mcBrdGetPhaseCurrentW)(void);
    mc_bool_t (*mcBrdCurrentCalibrationInit)(void);
    mc_bool_t (*mcBrdCurrentCalibrationHandler)(void);

    mc_float_t (*mcBrdGetDcBusVoltage)(void);
    mc_bool_t (*mcBrdGetDCBusOpenStatus)(void);

    mctemp_t (*mcBrdGetIpmTemperature)(void);
    mc_bool_t (*mcBrdGetIpmFaultOutState)(void);
    void (*mcBrdResetIpmFaultOutState)(void);

    void (*mcBrdSetPwmDuties)(mc_float_t dutyU, mc_float_t dutyV, mc_float_t dutyW);
    void (*mcBrdShortCircBotTransistors)(void);

    mc_bool_t (*mcBrdGetEncoderData)(mc_float_t *_pos, mc_float_t *_sp);
} mcMotorFunc_t;        ///< functions used by a specific motor

typedef struct{
    mc_control_state_t control_state;       ///< variable that shows current motor control state
    mc_bool_t parameterCheckFlag;           ///< shows if any parameter is corrected or modified while parameter initialization

    const mcCp_t *p_flash;    ///< flash address for related motor parameters
    
    mcCp_t p;           ///< parameters
    mcCv_t v;           ///< variables
    
    mindx_t mindx;     ///< motor index
    
    mcMotorFunc_t *func;         ///< functions used by a specific motor
    
    htck_t sysHandlerTotalCntFL;                        ///< System fast loop total counter as a unit of system handler calls
    htck_t sysHandlerTotalCntSL;                        ///< System fast loop total counter as a unit of system handler calls
    htck_t sysHandlerTotalCntML;                        ///< System main loop total counter as a unit of system handler calls

    mccnt_t rotRequestCntr;     ///< rotation request counter
} mcCpv_t;      ///< Control parameters and variables for a motor

typedef struct{
    mc_bool_t Initialized;
    mindx_t motorIndex;
    
    mcFunc_t func;      ///< functions used for all motor control module
} mc_t;

/** @}*/



#endif
