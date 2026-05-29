#ifndef FSTYPES_H
#define FSTYPES_H


/******************************************************************************/
typedef struct{
    mc_bool_t flag;                        /* flag that shows if any over current fault is active */
    mc_bool_t faultFlagPhaseCurrent;       /* flag that shows if phase current fault is active */
    mc_bool_t faultFlagVectorCurrent;      /* flag that shows if current vector fault is active */
    mc_uint16_t phaseCurrentErrorCntr;     /* instant error counter for over phase current */
    mc_uint16_t vector2CurrentErrorCntr;   /* instant error counter for over vector current */
    mc_uint16_t phaseCurrentFaultCntr;     /* instant fault counter for over phase current */
    mc_uint16_t vector2CurrentFaultCntr;   /* instant fault counter for over vector current */
} fsOverCurrentVar_t;                   /* variables for over current check */

typedef struct{
    mc_bool_t enabled;                     /* flag that shows if over current fault is enabled */
    mcc_t maxPhaseCurrent;              /* permitted instant maximum phase current */
    mcc_t maxVector2Current;            /* permitted instant maximum square vector value */
    mc_uint16_t phaseCurrentErrorCntrsMax; /* maximum permitted instant error for over phase current */
    mc_uint16_t vector2CurrentErrorCntrMax;/* maximum permitted instant error for over vector current */
    mc_uint16_t phaseCurrentFaultCntrMax;  /* maximum permitted instant fault for over phase current */
    mc_uint16_t vector2CurrentFaultCntrMax;/* maximum permitted instant fault for over vector current */
    htck_t duration;                    /* wait duration when ver current fault is active */
} fsOverCurrentPar_t;                   /* parameters for over current check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                     /* flag that shows if any current integral fault is active */
    mcc_t intg;                         /* integral of sampled current vector amplitude square */
    mccnt_t lowCurrentCntr;             /* conuters that counts if currect vector square is less than determined minimum current */
    mccnt_t windowCntr;                 /* counter that decides if enough samples are obtained */
    mccnt_t faultCntr;                  /* counts generated faults for current integral check */
} fsCurrentIntegralVar_t;

typedef struct{
    mc_bool_t enabled;                  /* flag that shows if current integral fault is enabled */
    mcc_t minCurrent;                   /* minimum current vector square value for mechanism activation */
    mcc_t intgMax;                      /* permitted maximum integral value, then generates fault condition */
    mccnt_t lowCurrentCntrMax;          /* maximum counter to decide low current vector square */
    mccnt_t windowCntrMax;              /* decides if enough samples are provided for over current decision */
    mccnt_t faultCntrMax;               /* maximum fault generation for fatal fault state */
    htck_t duration;                    /* latency duration if any related fault is generated */
} fsCurrentIntegralPar_t;
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if current sampling fault is active */
    mcc_t sumOfPhCurrents;         /* sum of phase currents */
    mc_uint16_t errorCntr;         /* instant error counter for current sampling */
    mc_uint16_t faultCntr;         /* instant fault counter for current sampling */
} fsCurrentSamplingVar_t;       /* variables for current sampling check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if current sampling check is enabled */
    mcc_t sumOfPhCurrentsMax;   /* permitted maximum phase current sum */
    mc_uint16_t errorCntrMax;      /* maximum permitted error counter for current sampling */
    mc_uint16_t faultCntrMax;      /* maximum permitted counter for current sampling */
    htck_t duration;            /* wait duration when current sampling fault is active */
} fsCurrentSamplingPar_t;       /* parameters for current sampling check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that showsif any DQ current fault is active */
    mc_bool_t flagD;               /* flag that shows if target D current fault is active */
    mc_bool_t flagQ;               /* flag that shows if target D current fault is active */
    mcc_t errCurrentD;          /* D current error */
    mcc_t errCurrentQ;          /* Q current error */
    mc_uint16_t errorCntrD;        /* counter increases when D current error is more than predefined parameter value */
    mc_uint16_t errorCntrQ;        /* counter increases when Q current error is more than predefined parameter value */
    mc_uint16_t faultCntrD;        /* counter increases when D current handled fault is more than predefined parameter value */
    mc_uint16_t faultCntrQ;        /* counter increases when Q current handled fault is more than predefined parameter value */
} fsTargetCurrentVar_t;         /* variables for current sampling check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if target current check is enabled */
    mcc_t errCurrentMaxD;       /* predefined maximum permitted D current error */
    mcc_t errCurrentMaxQ;       /* predefined maximum permitted Q current error */
    mc_uint16_t errorCntrMax;      /* maximum permitted error counter for target current */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for target current */
    htck_t duration;            /* wait duration when target current fault is active */
} fsTargetCurrentPar_t;         /* parameters for target current check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if loss of phase fault is active for any phase */
    mc_bool_t flagPhU;             /* flag that shows if loss of phase fault is active for phase U */
    mc_bool_t flagPhV;             /* flag that shows if loss of phase fault is active for phase V */
    mc_bool_t flagPhW;             /* flag that shows if loss of phase fault is active for phase W */
    mc_uint16_t errorCntrPhU;      /* counter increases when U current is less then predefined value even U current is desired */
    mc_uint16_t errorCntrPhV;      /* counter increases when V current is less then predefined value even V current is desired */
    mc_uint16_t errorCntrPhW;      /* counter increases when W current is less then predefined value even W current is desired */
    mc_uint16_t faultCntrPhU;      /* counter increases when U current is less then predefined value even U current is desired */
    mc_uint16_t faultCntrPhV;      /* counter increases when V current is less then predefined value even V current is desired */
    mc_uint16_t faultCntrPhW;      /* counter increases when W current is less then predefined value even W current is desired */
    mcc_t iuvwMin;                 /* minimum threshold for each phase current */
    mcdeg_t th;                    /* actual phase current degree */
} fsLopVar_t;                      /* variables for loss of phase */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if target current check is enabled */
    mcc_t threCoeff;            /* coefficient to calculate phase current existence threshold value */
    mcc_t iamp_min;             /* minimum current vector value to inspect loss of phase condition */
    mc_bool_t check_th;         /* decide if theta is checked for loss of phase inspection */
    mcdeg_t NoCurrentPos1PhUmin;/* first position interval which no phase U current is expected; minimum angle for the intervel */
    mcdeg_t NoCurrentPos1PhUmax;/* first position interval which no phase U current is expected; maximum angle for the intervel */
    mcdeg_t NoCurrentPos2PhUmin;/* second position interval which no phase U current is expected; minimum angle for the intervel */
    mcdeg_t NoCurrentPos2PhUmax;/* second position interval which no phase U current is expected; maximum angle for the intervel */
    mcdeg_t NoCurrentPos1PhVmin;/* first position interval which no phase U current is expected; minimum angle for the intervel */
    mcdeg_t NoCurrentPos1PhVmax;/* first position interval which no phase U current is expected; maximum angle for the intervel */
    mcdeg_t NoCurrentPos2PhVmin;/* second position interval which no phase U current is expected; minimum angle for the intervel */
    mcdeg_t NoCurrentPos2PhVmax;/* second position interval which no phase U current is expected; maximum angle for the intervel */
    mcdeg_t NoCurrentPos1PhWmin;/* first position interval which no phase U current is expected; minimum angle for the intervel */
    mcdeg_t NoCurrentPos1PhWmax;/* first position interval which no phase U current is expected; maximum angle for the intervel */
    mcdeg_t NoCurrentPos2PhWmin;/* second position interval which no phase U current is expected; minimum angle for the intervel */
    mcdeg_t NoCurrentPos2PhWmax;/* second position interval which no phase U current is expected; maximum angle for the intervel */
    mc_uint16_t errorCntrMax;      /* maximum permitted error counter for target current */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for target current */
    htck_t duration;            /* wait duration when LOP fault is active */
} fsLopPar_t;                   /* parameters for loss of phase */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if target speed fault is active */
    mc_uint16_t faultCntr;         /* counter increases when target speed is set more than predefined value */
} fsTargetSpeedVar_t;           /* variables for target speed check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if target current check is enabled */
    mc_uint16_t maxSpeed;          /* predefined maximum target speed */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for target speed fault */
    htck_t duration;            /* wait duration when target speed fault is active */
} fsTargetSpeedPar_t;           /* parameters for target speed check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if reference speed fault is active */
    mc_uint16_t faultCntr;         /* counter increases when reference speed is set more than predefined value */
} fsRefSpeedVar_t;              /* variables for reference speed check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if reference current check is enabled */
    mc_uint16_t maxSpeed;          /* predefined maximum reference speed */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for reference speed */
    htck_t duration;            /* wait duration when reference speed fault is active */
} fsRefSpeedPar_t;              /* parameters for reference speed check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if estimated speed fault is active */
    mc_uint16_t errorCntr;         /* counter increases when estimated speed is set more than predefined value */
    mc_uint16_t faultCntr;         /* counter increases when fault handler is executed more than predefined value */
} fsEstSpeedVar_t;              /* variables for estimated speed check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if estimated current check is enabled */
    mc_uint16_t errorCntrMax;      /* maximum permitted error counter for estimated speed */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for estimated speed */
    htck_t duration;            /* wait duration when speed estimation fault is active */
} fsEstSpeedPar_t;              /* parameters for estimated speed check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if speed controller fault is active */
    mc_uint16_t errorCntr;         /* counter increases when speed controller can not set current speed to refernce speed */
    mc_uint16_t faultCntr;         /* counter increases when fault handler is executed more than predefined value */
} fsSpControllerVar_t;          /* variables for speed controller check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if speed controller check is enabled */
    mcsp_t speedErrorMax;       /* permitted maximum speed error */
    mc_uint16_t errorCntrMax;      /* maximum permitted error counter for speed controller */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for speed controller */
    htck_t duration;            /* wait duration when speed controller fault is active */
} fsSpControllerPar_t;          /* parameters for speed controller check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;             /* flag that shows if on the fly duration fault is active */
    htck_t cntr;                /* counter that counts on the fly duration */
    mc_uint16_t faultCntr;      /* counter increases when fault handler is executed more than predefined value */
} fsOnTheFlyDurationVar_t;

typedef struct{
    mc_bool_t enabled;          /* flag that shows if on the fly duration check is enabled */
    htck_t maxStateDuration;    /* permitted maximum on the fly state duration */
    mc_uint16_t faultCntrMax;   /* maximum permitted fault counter for on the fly duration check */
    htck_t faultDuration;       /* wait duration when on the fly duration fault is active */
} fsOnTheFlyDurationPar_t;
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;             /* flag that shows if startup open loop duration fault is active */
    htck_t cntr;                /*  counter that counts startup duration */
    mc_uint16_t faultCntr;      /* counter increases when fault handler is executed more than predefined value */
} fsStartupOlDurationVar_t;          /* variables for startup open loop duration check */

typedef struct{
    mc_bool_t enabled;          /* flag that shows if startup open loop duration is enabled */
    htck_t maxDuration;         /* permitted maximum startup open loop duration */
    mc_uint16_t faultCntrMax;   /* maximum permitted fault counter for startup open loop duration check */
    htck_t duration;            /* wait duration when startup open loop duration fault is active */
} fsStartupOlDurationPar_t;          /* parameters for startup open loop duration check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;             /* flag that shows if turnoff open loop duration fault is active */
    htck_t cntr;                /* counter that counts turn off duration */
    mc_uint16_t faultCntr;      /* counter increases when fault handler is executed more than predefined value */
} fsTurnoffOlDurationVar_t;          /* variables for turnoff open loop duration check */

typedef struct{
    mc_bool_t enabled;          /* flag that shows if turnoff open loop duration is enabled */
    htck_t maxDuration;         /* permitted maximum turnoff open loop duration */
    mc_uint16_t faultCntrMax;   /* maximum permitted fault counter for turnoff open loop duration check */
    htck_t duration;            /* wait duration when turnoff open loop duration fault is active */
} fsTurnoffOlDurationPar_t;          /* parameters for turnoff open loop duration check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if dc bus voltage fault is active */
    mc_uint16_t maxErrorCntr;      /* counter increases when dc bus voltage is more than permitted maximum value */
    mc_uint16_t minErrorCntr;      /* counter increases when dc bus voltage is less than permitted minimum value */
    mc_uint16_t openErrorCntr;      /* counter increases when dc bus voltage is less than permitted minimum value */
    mc_uint16_t faultCntr;         /* counter increases when fault handler is executed more than predefined value */
} fsDcBusVoltageVar_t;          /* variables for speed controller check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if speed controller check is enabled */
    mcv_t maxDcBus;             /* permitted maximum dc bus voltage */
    mcv_t minDcBus;             /* permitted minimum dc bus voltage */
    mc_uint16_t errorCntrMax;      /* maximum permitted error counter for speed controller */
    mc_uint16_t errorCntrPinOpenMax;      /* maximum permitted error counter for speed controller */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for speed controller */
    htck_t duration;            /* wait duration when dc bus voltage level fault is active */
} fsDcBusVoltagePar_t;          /* parameters for speed controller check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                     /* flag that shows if any VBUS integral fault is active */
    mcc_t intg;                         /* integral of VBUS */
    mccnt_t lowVoltageCntr;             /* conuters that counts if VBUS is less than determined minimum voltage */
    mccnt_t windowCntr;                 /* counter that decides if enough samples are obtained */
    mccnt_t faultCntr;                  /* counts generated faults for VBUS integral check */
} fsDcBusIntegralVar_t;

typedef struct{
    mc_bool_t enabled;                  /* flag that shows if VBUS integral fault is enabled */
    mcc_t minVoltage;                   /* minimum VBUS value for mechanism activation */
    mcc_t intgMax;                      /* permitted maximum integral value, then generates fault condition */
    mccnt_t lowVoltageCntrMax;          /* maximum counter to decide low VBUS */
    mccnt_t windowCntrMax;              /* decides if enough samples are provided for high VBUS decision */
    mccnt_t faultCntrMax;               /* maximum fault generation for fatal fault state */
    htck_t duration;                    /* latency duration if any related fault is generated */
} fsDcBusIntegralPar_t;
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                 /* flag that shows if Ipm high temperature fault is active */
    mc_uint16_t faultCntr;          /* counter increases when fault handler is executed more than predefined value */
} fsIpmTempVar_t;                   /* variables for Ipm high temperature check */

typedef struct{
    mc_bool_t enabled;              /* flag that shows if Ipm high temperature check is enabled */
    mctemp_t maxTemp;               /* maximum permitted ipm temperature; fault is generated if exceeded */
    mc_uint16_t faultCntrMax;       /* maximum permitted fault counter for Ipm high temperature check */
    htck_t duration;                /* wait duration when ipm high temperature fault is active */
} fsIpmTempPar_t;                   /* parameters for Ipm high temperature check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if Ipm Fault Out Pin fault is active */
    mc_uint16_t faultCntr;         /* counter increases when fault handler is executed more than predefined value */
} fsIpmFaultOutVar_t;           /* variables for Ipm Fault Out Pin check */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if Ipm Fault Out Pin check is enabled */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for Ipm Fault Out Pin check */
    htck_t duration;            /* wait duration when ipm fault out fault is active */
} fsIpmFaultOutPar_t;           /* parameters for Ipm Fault Out Pin check */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;         /* flag that shows if over load fault is just generated */
    mcc_t intg;             /* current vector square integral value */
    mccnt_t faultCntr;      /* counter that counts generated over load faults */
} fsOverLoadVar_t;

typedef struct{
    mc_bool_t enabled;      /* parameters that decides if over load check is enabled */
    mcc_t intgMax;          /* maximum permitted integral value; fault is generated if exceeded */
    mcc_t minCurrent;       /* minimum current that mechanism is under calculation */
    mccnt_t faultCntrMax;   /* maximum permitted fault generation for fatal state */
    htck_t duration;        /* delay duration for fault state */
} fsOverLoadPar_t;
/******************************************************************************/

/******************************************************************************/
typedef struct{
    mc_bool_t flag;                /* flag that shows if thermal protection fault is active */
    mc_uint16_t faultCntr;         /* counter increases when fault handler is executed more than predefined value */

    /********* Variable for initial temperature calculation via resistance ****/
    mcv_t v;                    /* applied voltage */
    mcc_t i;                    /* sampled current */
    mcv_t v_sum;                /* sum of applied voltages for resistance calculation */
    mcc_t i_sum;                /* sum of sampled currents */
    mcv_t r_sum;                /* sum of calculated resistances */

    mcv_t v1;                   /* average of applied voltages for resistance calculation */
    mcc_t i1;                   /* average of sampled currents */
    mcr_t r1;                   /* winding resistance calculated for 1st stage */
    mccnt_t scnt1;              /* counter value that is used for r1 calculation */
    
    mcv_t v2;                   /* average of applied voltages for resistance calculation */
    mcc_t i2;                   /* average of sampled currents */
    mcr_t r2;                   /* winding resistance calculated for 2nd stage */
    mccnt_t scnt2;              /* counter value that is used for r2 calculation */
    
    mccnt_t scnt;               /* sample counter for resistance calculation */
    mcr_t r;                    /* winding resistance calculated */
    mcr_t rfinal;                    /* winding resistance calculated finally */
    mcv_t voffset1;             /* offset voltage (cause of dead-time) calculated for first thermal alignment */
    mcv_t voffset2;             /* offset voltage (cause of dead-time) calculated for first thermal alignment */
    mcv_t voffset;              /* offset voltage (cause of dead-time) decided for the following equitions */

    /*********** Variables for run time termal calculations *******************/
    mcuvw_t iuvw;               /* sampled phase currents */
    mcuvw_t i2uvw_sum;           /* sum of sampled phase currents */
    mcuvw_t i2uvw_ave;           /* average of sampled phase currents */

    mcr_t rcoeff;               /* resistance coefficient belongs to wire metarial */
    mcpow_t pcoeff_phase;       /* coefficient to apply power effect to each phase temperature */
    mctemp_t pDeltaTempMax;     /* maximum permitted phase delta temperature */
    mctemp_t mDeltaTempMax;     /* maximum permitted motor delta temperature */
    mcr_t rref;           /* reference total resistance for assumed reference temperature */
    
    mcuvw_t ruvw;               /* calculated phase resistances  */
    mcuvw_t puvw;               /* calculated resistance base consumed powers */
    mcuvw_t tdelta;             /* delta temperatures for each phase */
    
    mcpow_t ptotal;             /* resistance based total consumed power in all phases */
    mcpow_t plosses_coeff_motor;/* coefficient to calculate power effect on motor (rotor) temperature */
    mcpow_t peffect;            /* calculated power effect for motor (rotor) temperature */
    
    mctemp_t tempFocScale;      /* scale variable for parameter calculations */
    mctemp_t A2param;           /* parameter to apply negative instant temperature effect on temperature rise of motor */
    mcpow_t B1param;            /* coefficint to calculate finel rising power effect on motor temperature   */
    mctemp_t pHwPowerMax;       /* maximum hardware sampled power (ADC full scale dependent)  */

    mctemp_t mt_delta0;         /* calculated initial delta temperature of motor (rotor) */
    mctemp_t mt_delta;          /* calculated delta temperature of motor (rotor) */
    mctemp_t mt0;               /* calculated initial temperature of motor (rotor) */
    mctemp_t mt;                /* calculated temperature of motor (rotor) */
    mctemp_t A2washingcoeff;    /* coefficient for run time calculated instant motor temperature effect on calculated motor temperature  */

    mc_bool_t i2averaged;       /* flag that shows if current square data averaging process result is ready */
} fsThermalProtectionVar_t;     /* variables for thermal protection */

typedef struct{
    mc_bool_t enabled;             /* flag that shows if thermal protection is enabled */
    
    mccnt_t scnt_max;           /* maximum counter(Fast loop) to apply filter for sampled currents on run time */
    mctemp_t tref;              /* decided temperature for reference resistance measurement */
    mcr_t rref;                 /* reference resistance for assumed reference temperature */
    mcr_t rref_phase;           /* reference phase resistance for assumed reference temperature */
    mcr_t rref_cable;           /* reference connection cable resistance for assumed reference temperature */
    mcr_t wcoeff;               /* resistance coefficient depends on wire metarial */
    mcsp_t scoeff;              /* speed effect coefficient to power effect on motor (rotor) temperature */
    mctemp_t A2washingoffset;   /* offsetvalue for run time calculated instant motor temperature effect on calculated motor temperature  */
    mctemp_t A2washingcoeff;    /* coefficient for run time calculated instant motor temperature effect on calculated motor temperature  */

    mctemp_t mt_delta_min;      /* minimum calculated delta temperature of motor (rotor) */
    mctemp_t mt_delta_max;      /* maximum calculated delta temperature of motor (rotor) */

    mcpow_t currentLimitCompCoeff;      /* current limit compatibility coefficient to calculate power effect coefficient for each phase */
    mcpow_t tempCompCoeff;      /* temperature competibility cefficient to calculate B1 parameter */
    
    mctemp_t pRefPowerMax;      /* maximum phase power in reference resistance (estimated value)  */
    mcv_t maxSampledVoltage;    /* maximum voltage that can be sampled with the hardware */
    mcc_t maxSampledCurrent;    /* maximum current that can be sampled with the hardware */
    
    mctemp_t mTempMax;          /* maximum permitted motor temperature */
    mctemp_t pTempMax;          /* maximum permitted phase temperature */

    htck_t duration;            /* wait duration when thermal protection fault is active */
    mc_uint16_t faultCntrMax;      /* maximum permitted fault counter for yhermal protection */
} fsThermalProtectionPar_t;     /* parameters for thermal protection */

/******************************************************************************/

/******************************************************************************/
typedef enum{
    FS_STATE_NO_FAULT,
    FS_STATE_FAULT
} mc_fs_state_t;
/******************************************************************************/

/******************************************************************************/
typedef struct{
    const mcdq_t *idqref;       /* reference dq current */
    const mcdq_t *idq;          /* dq current feedback */
    const mcdq_t *vdq;          /* calculated reference dq phase voltage */

    const mcdeg_t *thivec;      /* reference current vector position; angle to U phase   */

    const mcsp_t *sprefin;      /* target motor speed pointer */
    const mcsp_t *spref;        /* reference motor speed pointer */
    const mcsp_t *spest;        /* estimated motor speed pointer */

    mcst_t *state;              /* motor control state pointer */

    fs_state_t *mc_state;        /* motor control functional safety states */
} fsMotorControlVar_t;          /* pointers for motor control variables */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    fsOverCurrentPar_t over_current;            /* parameters for over current check */
    fsCurrentIntegralPar_t current_integral;    /* parameters for current integral check */    
    fsCurrentSamplingPar_t current_sampling;    /* parameters for current sampling check */
    fsTargetCurrentPar_t target_current;        /* parameters for current sampling check */
    fsLopPar_t lop;                             /* parameters for loss of phase */

    fsTargetSpeedPar_t target_speed;            /* parameters for target speed check */
    fsRefSpeedPar_t ref_speed;                  /* parameters for reference speed check */
    fsEstSpeedPar_t est_speed;                  /* parameters for estimated speed check */
    fsSpControllerPar_t sp_controller;          /* parameters for speed controller check */
    fsOnTheFlyDurationPar_t onthefly_duration;  /* parameters for on the fly duration check */
    fsStartupOlDurationPar_t startupol_duration; /* parameters for startup open loop check */
    fsTurnoffOlDurationPar_t turnoffol_duration; /* parameters for turnoff open loop check */
    
    fsDcBusVoltagePar_t dcbus_voltage;          /* parameters for dc bara voltage level check */
    fsDcBusIntegralPar_t dcbus_integral;        /* parameters for dcbus integral check */

    fsIpmTempPar_t ipm_temp;                    /* parameters for ipm high temperature check */
    fsIpmFaultOutPar_t  ipm_fault_out;          /* parameters for ipm fault out pin check */

    fsOverLoadPar_t over_load;                  /* parameters for overload detection */
    fsThermalProtectionPar_t thermalProtection; /* parameters for thermal protection */

    htck_t noFaultCntrMax;             /* minimum no fault duration to reset fault counters */
} fsPar_t;                                      /* type for functional safety parameters */

typedef struct{
    mcuvw_t iuvw;
    mcuvw_t iuvw_abs;
    mcab_t iab;
    mcv_t vbus;
    mcc_t iamp2;
    mcc_t iamp;

    mc_bool_t ipm_faultout;                     /* ipm fault out pin state */
    mctemp_t ipm_temp_sampled;                  /* ipm temperature */

    fsMotorControlVar_t mcvars;                 /* pointers for motor control variables */
    
    fsOverCurrentVar_t over_current;            /* variables for over current check */
    fsCurrentIntegralVar_t current_integral;    /* variables for current integral check */
    fsCurrentSamplingVar_t current_sampling;    /* variables for current sampling check */
    fsTargetCurrentVar_t target_current;        /* variables for current sampling check */
    fsLopVar_t lop;                             /* variables for loss of phase */

    fsTargetSpeedVar_t target_speed;            /* variables for target speed check */
    fsRefSpeedVar_t ref_speed;                  /* variables for reference speed check */
    fsEstSpeedVar_t est_speed;                  /* variables for estimated speed check */
    fsSpControllerVar_t sp_controller;          /* variables for speed controller check */
    fsOnTheFlyDurationVar_t onthefly_duration;  /* variables for on the fly duration check */
    fsStartupOlDurationVar_t startupol_duration; /* variables for startup open loop check */
    fsTurnoffOlDurationVar_t turnoffol_duration; /* variables for turnoff open loop check */

    fsDcBusVoltageVar_t dcbus_voltage;          /* variables for dc bara voltage level check */
    fsDcBusIntegralVar_t dcbus_integral;        /* variables for dcbus integral check */
    
    fsIpmTempVar_t ipm_temp;                    /* variables for ipm high temperature check */
    fsIpmFaultOutVar_t ipm_fault_out;           /* variables for ipm fault out pin check */
    
    fsOverLoadVar_t over_load;                  /* variables for overload detection */
    fsThermalProtectionVar_t thermalProtection; /* variables for thermal protection */

    mc_bool_t fsFaultFlag;                         /* flags that shows if system is under any fault */
    mc_bool_t fsFatalFaultFlag;                    /* flags that shows if system is under fatal fault */
    mc_uint32_t fsTotalGeneratedFault;          /* counter shows total number of generated faults */
    mc_uint32_t generatedFaultsBinary;          /* binary boolean combined variable that keeps the latest generated fault type */
    
    htck_t noFaultCntr;                         /* counts duration that has no generated fault */

    htck_t halt_counter;                        /* delay counter that halts motor control and disables PWMs */
    htck_t halt_duration;                       /* decides delay time that halts motor control and disables PWMs */

    mc_fs_state_t mc_fs_state;                  /* triggered when fault state is generated and set for all fault state duration */

    mcps_t vbus_pin_circuit_state;              /* DC bus pin open circuit state */
} fsVar_t;                                      /* type for functional safety variables */
/******************************************************************************/

/******************************************************************************/
typedef struct{
    fsPar_t p;  /* functional safety parameters for the specified motor */
    fsVar_t v;  /* functional safety parameters for the specified motor */  

    mc_uint32_t fsHandlerCntFL;     /* functional safety fast loop counter */
    mc_uint32_t fsHandlerCntSL;     /* functional safety slow loop counter */

    mc_bool_t enabled;             /* decides if functional safety is disabled or enabled */
} fsSpv_t;      /* type includes functional safety parameters and variables */ 
/******************************************************************************/


#endif
