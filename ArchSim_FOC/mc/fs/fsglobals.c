#include "mcinclude.h"

fsSpv_t spv[SUPPORTED_TOTAL_MOTOR];

const fsPar_t fsp_default = {
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_FALSE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 4.0f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 16.0f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 4u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 4u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_FALSE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 1u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 16000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
         .threCoeff = LOP_MIN_CURRENT_COEFF,    /* mcc_t threCoeff */

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 200u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 18000u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 18000u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .speedErrorMax = 5.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */        
        .maxStateDuration = 0,      /* htck_t maxStateDuration */
        .faultCntrMax = 0,          /* mc_uint16_t faultCntrMax */
        .faultDuration = 0,         /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_FALSE,              /* mc_bool_t enabled */
        .maxDcBus = 400.0f,            /* mcv_t maxDcBus */
        .minDcBus = 100.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 300u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .maxTemp = 0.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 0u,         /* mc_uint16_t faultCntrMax */
        .duration = 2000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_FALSE,              /* mc_bool_t enabled */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_FALSE,     /* mc_bool_t enabled */
        .intgMax = 0.0f,        /* mcc_t intgMax */
        .minCurrent = 0.0f,     /* mcc_t minCurrent */
        .faultCntrMax = 0u,     /* mccnt_t faultCntrMax */
        .duration = 0u          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 16000,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    }    
};
#ifdef ENABLE_WM_ORCA
const fsPar_t fsp_wm_orca = {   /* WM orca motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 6.0f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 36.0f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 16u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 16u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 1u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mcc_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mcc_t errCurrentMaxQ */
        .errorCntrMax = 16000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_TRUE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 1000u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 18000u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7,              /* mc_uint16_t faultCntrMax */
        .duration = 2000    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .speedErrorMax = 150.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 32000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */        
        .maxStateDuration = 0,      /* htck_t maxStateDuration */
        .faultCntrMax = 0,          /* mc_uint16_t faultCntrMax */
        .faultDuration = 0,         /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 7000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 9000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .maxDcBus = 400.0f,            /* mcv_t maxDcBus */
        .minDcBus = 100.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 300u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .maxTemp = 0.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 0u,         /* mc_uint16_t faultCntrMax */
        .duration = 2000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_FALSE,     /* mc_bool_t enabled */
        .intgMax = 0.0f,        /* mcc_t intgMax */
        .minCurrent = 0.0f,     /* mcc_t minCurrent */
        .faultCntrMax = 0u,     /* mccnt_t faultCntrMax */
        .duration = 0u          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_TRUE,        /* bool_t enabled */
        .scnt_max = 24000u,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.00145f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 500.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    }
};
#endif
#ifdef ENABLE_DRY_ORCA
const fsPar_t fsp_dry_orca = {   /* WM orca motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 6.0f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 36.0f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 16u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 16u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_FALSE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 1u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mcc_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mcc_t errCurrentMaxQ */
        .errorCntrMax = 16000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 1000u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 18000u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7,              /* mc_uint16_t faultCntrMax */
        .duration = 2000    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .speedErrorMax = 150.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 32000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */        
        .maxStateDuration = 0,      /* htck_t maxStateDuration */
        .faultCntrMax = 0,          /* mc_uint16_t faultCntrMax */
        .faultDuration = 0,         /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 7000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 9000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_FALSE,              /* mc_bool_t enabled */
        .maxDcBus = 400.0f,            /* mcv_t maxDcBus */
        .minDcBus = 100.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 300u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .maxTemp = 0.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 0u,         /* mc_uint16_t faultCntrMax */
        .duration = 2000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_FALSE,              /* mc_bool_t enabled */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_FALSE,     /* mc_bool_t enabled */
        .intgMax = 0.0f,        /* mcc_t intgMax */
        .minCurrent = 0.0f,     /* mcc_t minCurrent */
        .faultCntrMax = 0u,     /* mccnt_t faultCntrMax */
        .duration = 0u          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_TRUE,        /* bool_t enabled */
        .scnt_max = 24000u,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.00145f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 500.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    }
};
#endif
#ifdef ENABLE_RAC_FAN
const fsPar_t fsp_rac_fan = {   /* RAC fan motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 1.2f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 1.44f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 4u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 4u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 1u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_TRUE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 16000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 200u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 1200u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 1200u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 5000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .speedErrorMax = 5.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_TRUE,         /* mc_bool_t enabled */        
        .maxStateDuration = 30000u, /* htck_t maxStateDuration */
        .faultCntrMax = 70u,        /* mc_uint16_t faultCntrMax */
        .faultDuration = 2000u,     /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 30000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_FALSE,              /* mc_bool_t enabled */
        .maxDcBus = 400.0f,            /* mcv_t maxDcBus */
        .minDcBus = 100.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 300u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .maxTemp = 0.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 0u,         /* mc_uint16_t faultCntrMax */
        .duration = 2000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_FALSE,     /* mc_bool_t enabled */
        .intgMax = 0.0f,        /* mcc_t intgMax */
        .minCurrent = 0.0f,     /* mcc_t minCurrent */
        .faultCntrMax = 0u,     /* mccnt_t faultCntrMax */
        .duration = 0u          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 16000u,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    }
};
#endif
#ifdef ENABLE_RAC_COMPRESSOR
const fsPar_t fsp_rac_compressor = {    /* RAC compressor motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 20.0f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 500.0f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 4u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 4u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7000u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7000u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7000u,      /* mc_uint16_t faultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 16000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7000u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u   /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 200u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7000u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 18000u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7000u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 18000,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 7000,              /* mc_uint16_t faultCntrMax */
        .duration = 2000    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7000,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .speedErrorMax = 500.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 1000,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7000,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */        
        .maxStateDuration = 0,      /* htck_t maxStateDuration */
        .faultCntrMax = 0,          /* mc_uint16_t faultCntrMax */
        .faultDuration = 0,         /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 10000,    /* htck_t maxDuration */
        .faultCntrMax = 7000u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .maxDcBus = 450.0f,            /* mcv_t maxDcBus */
        .minDcBus = 120.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 300u,             /* mc_uint16_t faultCntrMax */
        .duration = 10000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .maxTemp = 0.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 0u,         /* mc_uint16_t faultCntrMax */
        .duration = 2000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 7000u,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_FALSE,     /* mc_bool_t enabled */
        .intgMax = 0.0f,        /* mcc_t intgMax */
        .minCurrent = 0.0f,     /* mcc_t minCurrent */
        .faultCntrMax = 0u,     /* mccnt_t faultCntrMax */
        .duration = 0u          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 16000u,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    }
};
#endif
#ifdef ENABLE_REF_COMPRESSOR
const fsPar_t fsp_ref_compressor = {    				/* REF compressor motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 2.0f,//1.8f,      					/* mcc_t maxPhaseCurrent */
        .maxVector2Current = 4.0f,//3.24f,      				/* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 60u,     			/* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 60u,     		/* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 6u,      			/* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 6u,      			/* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 300000u    							/* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_TRUE,        /* mc_bool_t enabled */
        .minCurrent = 1.20f,         /* mcc_t minCurrent */
        .intgMax =3000.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 3000u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 5000u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 6u,         /* mccnt_t faultCntrMax */
        .duration = 300000u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_FALSE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 1u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 6000u,  						/* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u   /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 60u,                           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 6u,                             /* mc_uint16_t faultCntrMax */

        .duration = 300000u                            	/* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 4500u,          					/* mc_uint16_t maxSpeed */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .maxSpeed = 4500,          						/* mc_uint16_t maxSpeed */
        .faultCntrMax = 7,              /* mc_uint16_t faultCntrMax */
        .duration = 2000    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7,              /* mc_uint16_t faultCntrMax */
        .duration = 2000    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_FALSE,          /* mc_bool_t enabled */
        .speedErrorMax = 5.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 1000,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7,              /* mc_uint16_t faultCntrMax */
        .duration = 2000    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */        
        .maxStateDuration = 0,      /* htck_t maxStateDuration */
        .faultCntrMax = 0,          /* mc_uint16_t faultCntrMax */
        .faultDuration = 0,         /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_FALSE,              				/* mc_bool_t enabled */	/*appRefrigeratorHardwareStanbyControl �zerinden a��l�yor */
        .maxDcBus = 450.0f,            					/* mcv_t maxDcBus */
        .minDcBus = 120.0f,            					/* mcv_t minDcBus */ //85VAC->85*1.41-> 120VDC
        .errorCntrMax = 60u,           				/* mc_uint16_t errorCntrMax */ 	/*1000*/ /*1807*/
        .faultCntrMax = 6u,             				/* mc_uint16_t faultCntrMax */ 	/* 300000/166 */
        .duration = 300000u            					/* htck_t duration */			/*OVV_RESALL_SEC =300s -> 1ms SL->300000*/
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 430.0f,        	 				/* mcc_t minVoltage */
        .intgMax = 20000.0f,            					/* mcc_t intgMax */
        .lowVoltageCntrMax = 1000u,    					/* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 5000u,        					/* mccnt_t windowCntrMax */
        .faultCntrMax = 6u,         					/* mccnt_t faultCntrMax */
        .duration = 300000u              				/* htck_t duration */ //300000
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .maxTemp = 0.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 0u,         /* mc_uint16_t faultCntrMax */
        .duration = 2000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 6u,              				/* mc_uint16_t faultCntrMax */
        .duration = 300000u            					/* htck_t duration */ //6000->6sn
    },
    .over_load = {
        .enabled = MC_TRUE,     						/* mc_bool_t enabled */
        .intgMax = 3000.0f,        						/* mcc_t intgMax */
        .minCurrent = 1.20f,     						/* mcc_t minCurrent */
        .faultCntrMax = 6u,     						/* mccnt_t faultCntrMax */
        .duration = 300000u          						/* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 4500u,      						/* mccnt_t scnt_max */  /*6000*/
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    },
    .noFaultCntrMax = 3000u,   /* htck_t noFaultCntrMax */
};
#endif

//W20007335
#ifdef ENABLE_DW_CIRC
const fsPar_t fsp_dw_circ_W20007335 = {
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 0.95f,//0.9      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 0.9025f,//0.81      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 4u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 4u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 8000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_TRUE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */
		.iamp_min = 0.025f,

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 500u,                           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 4u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3500u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 10u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3500u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 10u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 2000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 10u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .speedErrorMax = 250.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 2000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,         /* mc_bool_t enabled */
        .maxStateDuration = 30000u, /* htck_t maxStateDuration */
        .faultCntrMax = 70u,        /* mc_uint16_t faultCntrMax */
        .faultDuration = 2000u,     /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 30000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .maxDcBus = 375.0f,            /* mcv_t maxDcBus */
        .minDcBus = 200.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 30000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 5000u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_TRUE,        /* mc_bool_t enabled */
        .maxTemp = 120.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 100u,         /* mc_uint16_t faultCntrMax */
        .duration = 20000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 2u,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_TRUE,     /* mc_bool_t enabled */
        .intgMax = 347416.0f,//100.0f , //347416.0f        /* mcc_t intgMax */
        .minCurrent = 0.2193f,//0.65f, //0.2193f,     /* mcc_t minCurrent */
        .faultCntrMax = 4u,     /* mccnt_t faultCntrMax */
        .duration = 180000u, //180000u,          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 7550u,      /* mccnt_t scnt_max */
        .tref = 20.0f,          /* mctemp_t tref */
        .rref_phase = 49.6f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.000001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 325.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 3.0f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    },
};
#endif

//W11652801
#ifdef ENABLE_DW_CIRC
const fsPar_t fsp_dw_circ_W11652801 = {   /* RAC fan motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 0.95f,//0.9      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 0.9025f,//0.81      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 4u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 4u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 7u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 7u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.2f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 8000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_TRUE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF,     /* mcc_t threCoeff */
		.iamp_min = 0.025f,

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 500u,                           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 4u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3500u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 10u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3500u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 10u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 2000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 10u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .speedErrorMax = 265.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 2000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,         /* mc_bool_t enabled */
        .maxStateDuration = 30000u, /* htck_t maxStateDuration */
        .faultCntrMax = 70u,        /* mc_uint16_t faultCntrMax */
        .faultDuration = 2000u,     /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 30000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .maxDcBus = 375.0f,            /* mcv_t maxDcBus */
        .minDcBus = 200.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 30000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 5000u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_TRUE,        /* mc_bool_t enabled */
        .maxTemp = 120.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 100u,         /* mc_uint16_t faultCntrMax */
        .duration = 20000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 2u,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_TRUE,     /* mc_bool_t enabled */
        .intgMax = 1082101.0f,//100.0f , //347416.0f        /* mcc_t intgMax */
        .minCurrent = 0.2025f,//0.65f, //0.2193f,     /* mcc_t minCurrent */
        .faultCntrMax = 4u,     /* mccnt_t faultCntrMax */
        .duration = 180000u, //180000u,          /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 7550u,      /* mccnt_t scnt_max */
        .tref = 20.0f,          /* mctemp_t tref */
        .rref_phase = 49.6f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.000001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 325.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 3.0f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    },
};
#endif

//W11402566
#ifdef ENABLE_DW_DRAIN
const fsPar_t fsp_dw_drain_W11402566 = {   /* RAC fan motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 0.7f,//0.7f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 0.49f,//0.49f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 7u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 7u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 10u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 10u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.25f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 8000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_TRUE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF_DRAIN,     /* mcc_t threCoeff */
		.iamp_min = 0.08f,

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 200u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3550u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3550u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .speedErrorMax = 300.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,         /* mc_bool_t enabled */
        .maxStateDuration = 30000u, /* htck_t maxStateDuration */
        .faultCntrMax = 70u,        /* mc_uint16_t faultCntrMax */
        .faultDuration = 2000u,     /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 30000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .maxDcBus = 375.0f,            /* mcv_t maxDcBus */
        .minDcBus = 200.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 30000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 5000u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_TRUE,        /* mc_bool_t enabled */
        .maxTemp = 120.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 100u,         /* mc_uint16_t faultCntrMax */
        .duration = 20000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 2u,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_TRUE,     /* mc_bool_t enabled */
        .intgMax = 13739.0f,//500.0f //13739.0f       /* mcc_t intgMax */
        .minCurrent = 0.0502f,//0.35f //0.0502f    /* mcc_t minCurrent */
        .faultCntrMax = 4u,     /* mccnt_t faultCntrMax */
        .duration = 150000u //2000u         /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 16000u,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    },
};
#endif

//W11377410
#ifdef ENABLE_DW_DRAIN
const fsPar_t fsp_dw_drain_W11377410 = {   /* RAC fan motor safety parameters */
    .over_current = {   /* fsOverCurrentPar_t over_current */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .maxPhaseCurrent = 0.75f,//0.7f,      /* mcc_t maxPhaseCurrent */
        .maxVector2Current = 0.5625f,//0.49f,      /* mcc_t maxVector2Current */
        .phaseCurrentErrorCntrsMax = 7u,     /* mc_uint16_t phaseCurrentErrorCntrsMax */
        .vector2CurrentErrorCntrMax = 7u,     /* mc_uint16_t vector2CurrentErrorCntrMax */
        .phaseCurrentFaultCntrMax = 10u,      /* mc_uint16_t phaseCurrentFaultCntrMax */
        .vector2CurrentFaultCntrMax = 10u,      /* mc_uint16_t vector2CurrentFaultCntrMax */
        .duration = 10000u    /* htck_t duration */
    },
    .current_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minCurrent = 0.0f,         /* mcc_t minCurrent */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowCurrentCntrMax = 0u,    /* mccnt_t lowCurrentCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .current_sampling = {   /* fsCurrentSamplingPar_t current_sampling */
        .enabled = MC_TRUE,   /* mc_bool_t enabled */
        .sumOfPhCurrentsMax = 0.25f,   /* mc_uint16_t sumOfPhCurrentsMax */
        .errorCntrMax = 16u,     /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .target_current = {   /* fsTargetCurrentPar_t target_current */
        .enabled = MC_FALSE,  /* mc_bool_t enabled */
        .errCurrentMaxD = 0.3f,    /* mc_uint16_t errCurrentMaxD */
        .errCurrentMaxQ = 0.3f,    /* mc_uint16_t errCurrentMaxQ */
        .errorCntrMax = 8000u,  /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 7u,      /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .lop = {   /* fsLopPar_t lop */
        .enabled = MC_TRUE,  /* mc_bool_t enabled */
        .threCoeff = LOP_MIN_CURRENT_COEFF_DRAIN,     /* mcc_t threCoeff */
		.iamp_min = 0.08f,

        .NoCurrentPos1PhUmin = NO_CURRENT_POS1_PHU_MIN,        /* mcdeg_t NoCurrentPos1PhUmin */
        .NoCurrentPos1PhUmax = NO_CURRENT_POS1_PHU_MAX,        /* mcdeg_t NoCurrentPos1PhUmax */
        .NoCurrentPos2PhUmin = NO_CURRENT_POS2_PHU_MIN,        /* mcdeg_t NoCurrentPos2PhUmin */
        .NoCurrentPos2PhUmax = NO_CURRENT_POS2_PHU_MAX,        /* mcdeg_t NoCurrentPos2PhUmax */

        .NoCurrentPos1PhVmin = NO_CURRENT_POS1_PHV_MIN,        /* mcdeg_t NoCurrentPos1PhVmin */
        .NoCurrentPos1PhVmax = NO_CURRENT_POS1_PHV_MAX,        /* mcdeg_t NoCurrentPos1PhVmax */
        .NoCurrentPos2PhVmin = NO_CURRENT_POS2_PHV_MIN,        /* mcdeg_t NoCurrentPos2PhVmin */
        .NoCurrentPos2PhVmax = NO_CURRENT_POS2_PHV_MAX,        /* mcdeg_t NoCurrentPos2PhVmax */

        .NoCurrentPos1PhWmin = NO_CURRENT_POS1_PHW_MIN,        /* mcdeg_t NoCurrentPos1PhWmin */
        .NoCurrentPos1PhWmax = NO_CURRENT_POS1_PHW_MAX,        /* mcdeg_t NoCurrentPos1PhWmax */
        .NoCurrentPos2PhWmin = NO_CURRENT_POS2_PHW_MIN,        /* mcdeg_t NoCurrentPos2PhWmin */
        .NoCurrentPos2PhWmax = NO_CURRENT_POS2_PHW_MAX,        /* mcdeg_t NoCurrentPos2PhWmax */

        .errorCntrMax = 200u,                            /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,                              /* mc_uint16_t faultCntrMax */

        .duration = 2000u                            /* htck_t duration */
    },
    .target_speed = {   /* fsTargetSpeedPar_t target_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3550u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .ref_speed = {   /* fsRefSpeedPar_t ref_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .maxSpeed = 3550u,          /* mc_uint16_t maxSpeed */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .est_speed = {   /* fsEstSpeedPar_t est_speed */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .sp_controller = {   /* fsSpControllerPar_t sp_controller */
        .enabled = MC_TRUE,          /* mc_bool_t enabled */
        .speedErrorMax = 300.0f,              /* mcsp_t speedErrorMax */
        .errorCntrMax = 1000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 3u,              /* mc_uint16_t faultCntrMax */
        .duration = 2000u    /* htck_t duration */
    },
    .onthefly_duration = {
        .enabled = MC_FALSE,         /* mc_bool_t enabled */
        .maxStateDuration = 30000u, /* htck_t maxStateDuration */
        .faultCntrMax = 70u,        /* mc_uint16_t faultCntrMax */
        .faultDuration = 2000u,     /* htck_t faultDuration */
    },
    .startupol_duration = {
        .enabled = MC_TRUE,    /* mc_bool_t enabled */
        .maxDuration = 30000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .turnoffol_duration = {
        .enabled = MC_FALSE,    /* mc_bool_t enabled */
        .maxDuration = 3000,    /* htck_t maxDuration */
        .faultCntrMax = 7u,     /* mc_uint16_t faultCntrMax */
        .duration = 2000u       /* htck_t duration */
    },
    .dcbus_voltage = {   /* fsDcBusVoltagePar_t dcbus_voltage */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .maxDcBus = 375.0f,            /* mcv_t maxDcBus */
        .minDcBus = 200.0f,            /* mcv_t minDcBus */
        .errorCntrMax = 30000u,           /* mc_uint16_t errorCntrMax */
        .faultCntrMax = 5000u,             /* mc_uint16_t faultCntrMax */
        .duration = 2000u            /* htck_t duration */
    },
    .dcbus_integral = {
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minVoltage = 0.0f,         /* mcc_t minVoltage */
        .intgMax = 0.0f,            /* mcc_t intgMax */
        .lowVoltageCntrMax = 0u,    /* mccnt_t lowVoltageCntrMax */
        .windowCntrMax = 0u,        /* mccnt_t windowCntrMax */
        .faultCntrMax = 0u,         /* mccnt_t faultCntrMax */
        .duration = 0u              /* htck_t duration */
    },
    .ipm_temp = {                   /* fsIpmTempPar_t ipm_temp */
        .enabled = MC_TRUE,        /* mc_bool_t enabled */
        .maxTemp = 120.0f,            /*  mctemp_t maxTemp */
        .faultCntrMax = 100u,         /* mc_uint16_t faultCntrMax */
        .duration = 20000u           /* htck_t duration */
    },
    .ipm_fault_out = {   /* fsIpmFaultOutPar_t  ipm_fault_out */
        .enabled = MC_TRUE,              /* mc_bool_t enabled */
        .faultCntrMax = 2u,              /* mc_uint16_t faultCntrMax */
        .duration = 10000u            /* htck_t duration */
    },
    .over_load = {
        .enabled = MC_TRUE,     /* mc_bool_t enabled */
        .intgMax = 75217.0f,//500.0f //13739.0f       /* mcc_t intgMax */
        .minCurrent = 0.0631f,//0.35f //0.0502f    /* mcc_t minCurrent */
        .faultCntrMax = 4u,     /* mccnt_t faultCntrMax */
        .duration = 150000u //2000u         /* htck_t duration */
    },
    .thermalProtection = {   /* fsThermalProtectionPar_t thermalProtection */
        .enabled = MC_FALSE,        /* bool_t enabled */
        .scnt_max = 16000u,      /* mccnt_t scnt_max */
        .tref = 25.0f,          /* mctemp_t tref */
        .rref_phase = 5.4f,           /* mcr_t rref */
        .rref_cable = 0.2f,           /* mcr_t rref */
        .wcoeff = 0.00393f,     /* mcr_t wcoeff */
        .scoeff = -0.00001f,     /* mcsp_t scoeff */
        .A2washingoffset = 0.00131f,     /* mctemp_t A2washingoffset */
        .A2washingcoeff = 0.000003118f,  /* mctemp_t A2washingcoeff */
        .mt_delta_min = -25.0f,         /* mctemp_t mt_delta_min */
        .mt_delta_max = 250.0f,         /* mctemp_t mt_delta_max */
        .currentLimitCompCoeff = 1.8f,  /* mcpow_t currentLimitCompCoeff */
        .tempCompCoeff = 1.52f,         /* mcpow_t tempCompCoeff */
        .pRefPowerMax = 100.0f,         /* mctemp_t pRefPowerMax */
        .maxSampledVoltage = 480.0f,    /* mcv_t maxSampledVoltage */
        .maxSampledCurrent = 9.2f,      /* mcv_t maxSampledCurrent */
        .mTempMax = 155.0f,     /* mctemp_t mTempMax */
        .pTempMax = 170.0f,     /* mctemp_t pTempMax */
        .duration = 120000u,     /* htck_t duration */
        .faultCntrMax = 7u       /* uint16_t faultCntrMax */
    },
};
#endif

const fsPar_t* fsp_px[SUPPORTED_TOTAL_MOTOR] = {
      #ifdef ENABLE_WM_ORCA
      &fsp_wm_orca,
      #endif
      #ifdef ENABLE_DRY_ORCA
      &fsp_dry_orca,
      #endif
      #ifdef ENABLE_RAC_COMPRESSOR
      &fsp_rac_compressor,
      #endif
      #ifdef ENABLE_RAC_FAN
      &fsp_rac_fan,
      #endif
	  #ifdef ENABLE_DW_CIRC
	  &fsp_dw_circ_W20007335,
	  #endif
	  #ifdef ENABLE_DW_DRAIN
	  &fsp_dw_drain_W11402566,
	  #endif
      #ifdef ENABLE_REF_COMPRESSOR
      &fsp_ref_compressor,
      #endif
};

