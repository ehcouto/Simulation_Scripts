#include "mcinclude.h"
/************************ Variable object definitions - start *****************/
mc_t mc = {0,};                 /* general object for all motor control process */
mcCpv_t mpv[SUPPORTED_TOTAL_MOTOR];

const mcCp_t mcp_default = {    /* default motor control parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 4u,      /* mc_uint8_t pp */
        .r = 6.5f,   /* mcr_t r */
        .Ld = 0.018f,  /* mcmp_t Ld */
        .Lq = 0.018f,  /* mcmp_t Lq */
        .Ke = 0.1f,  /* mcmp_t Ke */
    },
    .freqFL = 16000u,              /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 18000.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
	.minsp_c2o = 210.0f,		    /* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d = 3.0f,           /* idrefmax */
        .q = 4.0f            /* iqrefmax */
    },    
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 200.0f,         /* vdrefmax */
        .q = 200.0f          /* vqrefmax */
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 150.0f,       /* rampmaxAcc */
        .rampminAcc = 10.0f,        /* rampminAcc */
        .rampmaxDec = 150.0f,       /* rampmaxDec */
        .rampminDec = 10.0f         /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 2250,      /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
        .kp = 6.957f,         /* kp */
        .ki = 1968.0f,         /* ki */
        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
        .kp = 10.654f,           /* kp */
        .ki = 2560.0f,          /* ki */
        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_VCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 160.0f,         /* d component */
        .q = 160.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
        .d = 19680.0f,         /* d component */
        .q = 19680.0f,         /* q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
        .deadtime = 0.00792f,                   /* duty_t deadtime */
        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 0.0f,   /* d current */
                .q = 1.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */     
                .d = 0.0f,   /* d current */
                .q = 1.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */     
                .d = 0.0f,   /* d current */
                .q = 1.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 500.0f,        /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */     
                .d = 0.0f,   /* d current */
                .q = 1.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */     
                .d = 0.001f,    /* d current ramp */
                .q = 0.001f,    /* q current ramp */
            },
            .mecalign1st_duration = 1000.0f,       /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */    
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */     
                .d = 0.001f,    /* d current ramp */
                .q = 0.001f,    /* q current ramp */
            },
            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 0.0f,                      /* d current */
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    		            /* d current ramp */
                .q = 0.0f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idramp = 0.0f,       /*rmpc_t idramp */
            .iqramp = 0.001f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 0.0f,   /* idmin */
                .q = 1.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 0.0f,   /* idmax */
                .q = 2.0f    /* iqmax */
            },
            .sprefmax = 600.0f,         /* mcsp_t sprefmax */
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
            .minValidSp = 210.0f,     /* mcsp_t minValidSp */
            .tValidSp = (htck_t)100,       /* msec_t tValidSp */
            .tmax = 3.0f        /* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_DIRECT,         /* tmode_t mode */
        .duration = 3.0f,           /* sec_t duration */
        .thErrMin = -360.0f,        /* mcdeg_t thErrMin */
        .thErrMax = 360.0f          /* mcdeg_t thErrMax */
    },
    .disPwmWhenStop = MC_FALSE,
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.0f,   /* mcc_t d */
                .q = 1.5f,   /* mcc_t q */       
            },
            .idq1 = {   /* mcdq_t idq1 */
                .d = 0.0f,   /* mcc_t d */
                .q = 2.5f,   /* mcc_t q */       
            },
            .idq2 = {   /* mcdq_t idq2 */
                .d = 0.0f,   /* mcc_t d */  
                .q = 1.5f,   /* mcc_t q */       
            },
            .idq3 = {   /* mcdq_t idq3 */
                .d = 0.0f,   /* mcc_t d */ 
                .q = 1.0f,   /* mcc_t q */        
            },
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .t_rot = 5000.0f,           /* msec_t t_rot */
            .t1 = 3000.0f,           /* msec_t t1 */
            .t2 = 2000.0f,           /* msec_t t2 */
            .t3 = 1000.0f,           /* msec_t t3 */
            .dispwm = MC_FALSE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 0.0f,
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.0f,   /* mcc_t d */
        .q = 0.8f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
        .spmin = 4000.0f,        /* mcsp_t spmin */
        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.00005f,             /* kp */
            .ki = 1.28f,         /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 3.0f           /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minsp = 0.0f,              /* mcsp_t minsp */
        .maxsp = 0.0f,              /* mcsp_t maxsp */
        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */
        .duration = 200     /* mccnt_t duration */            
    },
    .bratio = 15.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 100,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -117.6548f, /* L11 */
            .ki = -18739.2f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -0.6876f,   /* L12 */
            .ki = 17.6f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 0.6239f,    /* L13 */
            .ki = -27.2f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaIq = {       /* mcpPiController_t edeltaIq */
            .kp = -117.3253f, /* L14 */
            .ki = -18739.2f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 210.0f,         /* mcsp_t minsp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.04f,          /* mcv_t k2 */
        .smc = 1600.0f,           /* mcv_t smc */
        .k1min = 2000.0f,        /* mcv_t k1min */
        .k1max = 75000.0f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_FALSE,    /* mcstate_t enabled */
        .minsp = 1800.0f,          /* mcsp_t minsp */
        .maxsp = 5250.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_FALSE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 0.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 0.0f,
    .pos_begin = 0.0f,
    .pos_mid = 0.0f,
    .pos_end = 0.0f,
    },
};
#ifdef ENABLE_WM_ORCA
const mcCp_t mcp_wm_orca = {    /* WM orca motor drive parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 4u,      /* mc_uint8_t pp */
        .r = 6.5f,   /* mcr_t r */
        .Ld = 0.018f,  /* mcmp_t Ld */
        .Lq = 0.018f,  /* mcmp_t Lq */
        .Ke = 0.1f,  /* mcmp_t Ke */
    },
    .freqFL = 8000u,              /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 18000.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 210.0f,            /* mcsp_t minsp_o2c */
	.minsp_c2o = 210.0f,            /* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d = 3.9f,           /* idrefmax */
        .q = 5.4f            /* iqrefmax */
    },    
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 200.0f,         /* vdrefmax */
        .q = 200.0f          /* vqrefmax */
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 300.0f,       /* rampmaxAcc */
        .rampminAcc = 75.0f,        /* rampminAcc */
        .rampmaxDec = 300.0f,       /* rampmaxDec */
        .rampminDec = 75.0f         /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.011389f,      /* kp */
        .ki = 0.0961f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 1800.0f,   /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 3000.0f,  /* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
        .kp = 6.957f,         /* kp */
        .ki = 1968.0f,         /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
        .kp = 10.654f,           /* kp */
        .ki = 2560.0f,          /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_VCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
        .deadtime = 0.00792f,               /* duty_t deadtime */
        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 2.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */    
                .d = 2.5f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */     
                .d = 3.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 500.0f,        /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */   
                .d = 3.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */     
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
                .d = 3.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */     
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 0.0f,                      /* d current */
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    		            /* d current ramp */
                .q = 0.0f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idramp = 0.0008f,       /*rmpc_t idramp */
            .iqramp = 0.0f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 3.0f,   /* idmin */
                .q = 0.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 3.8f,   /* idmax */
                .q = 0.0f    /* iqmax */
            },
            .sprefmax = 600.0f,         /* mcsp_t sprefmax */
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
            .minValidSp = 210.0f,     /* mcsp_t minValidSp */
            .tValidSp = (htck_t)250.0f,       /* msec_t tValidSp */
            .tmax = 3.0f        /* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
        .duration = 0.1f,           /* sec_t duration */
        .thErrMin = -360.0f,        /* mcdeg_t thErrMin */
        .thErrMax = 360.0f          /* mcdeg_t thErrMax */
    },
    .disPwmWhenStop = MC_FALSE,
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.5f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */      
            },
            .idq1 = {   /* mcdq_t idq1 */
                0.4f,   /* mcc_t d */
                0.0f,   /* mcc_t q */     
            },
            .idq2 = {   /* mcdq_t idq2 */
                0.3f,   /* mcc_t d */
                0.0f,   /* mcc_t q */      
            },
            .idq3 = {   /* mcdq_t idq3 */
                0.2f,   /* mcc_t d */  
                0.0f,   /* mcc_t q */        
            },
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .t_rot = 5000.0f,           /* msec_t t_rot */
            .t1 = 0.0f,           /* msec_t t1 */
            .t2 = 0.0f,           /* msec_t t2 */
            .t3 = 0.0f,           /* msec_t t3 */
            .dispwm = MC_FALSE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 0.0f,
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.1f,   /* mcc_t d */
        .q = 0.0f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.83f,           /*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 1600.0f,           /* vmarg_lpfcoeff */
        .spmin = 4000.0f,        /* mcsp_t spmin */
        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.00005f,             /* kp */
            .ki = 0.08f,         /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 2.7f           /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minsp = 0.0f,              /* mcsp_t minsp */
        .maxsp = 0.0f,              /* mcsp_t maxsp */
        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */ 
        .duration = 200u     /* mccnt_t duration */            
    },
    .bratio = 15.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 100u,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -117.6548f, /* L11 */
            .ki = -18739.2f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -0.6876f,   /* L12 */
            .ki = 17.6f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 0.6239f,    /* L13 */
            .ki = -27.2f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaIq = {       /* mcpPiController_t edeltaIq */
            .kp = -117.3253f, /* L14 */
            .ki = -18739.2f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 210.0f,         /* mcsp_t minsp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.04f,          /* mcv_t k2 */
        .smc = 1600.0f,           /* mcv_t smc */
        .k1min = 2000.0f,        /* mcv_t k1min */
        .k1max = 75000.0f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_TRUE,    /* mcstate_t enabled */
        .minsp = 3150.0f,          /* mcsp_t minsp */
        .maxsp = 3750.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_FALSE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 0.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 0.0f,
    .pos_begin = 0.0f,
    .pos_mid = 0.0f,
    .pos_end = 0.0f,
    },
};
#endif
#ifdef ENABLE_DRY_ORCA
const mcCp_t mcp_dry_orca = {    /* WM orca motor drive parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 4u,      /* mc_uint8_t pp */
        .r = 6.5f,   /* mcr_t r */
        .Ld = 0.018f,  /* mcmp_t Ld */
        .Lq = 0.018f,  /* mcmp_t Lq */
        .Ke = 0.1f,  /* mcmp_t Ke */
    },
    .freqFL = 16000u,              /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 18000.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 210.0f,            /* mcsp_t minsp_o2c */
	.minsp_c2o = 210.0f,		    /* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d =4.5f,           /* idrefmax */
        .q = 5.5f            /* iqrefmax */
    },
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 200.0f,         /* vdrefmax */
        .q = 200.0f          /* vqrefmax */
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 300.0f,       /* rampmaxAcc */
        .rampminAcc = 75.0f,        /* rampminAcc */
        .rampmaxDec = 300.0f,       /* rampmaxDec */
        .rampminDec = 75.0f         /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.011389f,      /* kp */
        .ki = 0.0961f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
        .kp = 0.0056314f,      /* kp */
        .ki = 0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 1800.0f,   /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 30000.0f,  /* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
        .kp = 6.957f,         /* kp */
        .ki = 1968.0f,         /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
        .kp = 10.654f,           /* kp */
        .ki = 2560.0f,          /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_VCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
        .deadtime = 0.00792f,               /* duty_t deadtime */
        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 2.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */    
                .d = 2.5f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */     
                .d = 3.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 500.0f,        /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */   
                .d = 1.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
                .d = 1.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 0.0f,                      /* d current */
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    		            /* d current ramp */
                .q = 0.0f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idramp = 0.0008f,       /*rmpc_t idramp */
            .iqramp = 0.0f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 3.0f,   /* idmin */
                .q = 0.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 1.0f,   /* idmax */
                .q = 0.0f    /* iqmax */
            },
            .sprefmax = 6000.0f,         /* mcsp_t sprefmax */
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
            .minValidSp = 210.0f,     /* mcsp_t minValidSp */
            .tValidSp = (htck_t)250.0f,       /* msec_t tValidSp */
            .tmax = 3.0f        /* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
        .duration = 0.1f,           /* sec_t duration */
        .thErrMin = -360.0f,        /* mcdeg_t thErrMin */
        .thErrMax = 360.0f          /* mcdeg_t thErrMax */
    },
    .disPwmWhenStop = MC_FALSE,
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.5f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq1 = {   /* mcdq_t idq1 */
                0.4f,   /* mcc_t d */
                0.0f,   /* mcc_t q */
            },
            .idq2 = {   /* mcdq_t idq2 */
                0.3f,   /* mcc_t d */
                0.0f,   /* mcc_t q */
            },
            .idq3 = {   /* mcdq_t idq3 */
                0.2f,   /* mcc_t d */
                0.0f,   /* mcc_t q */
            },
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .t_rot = 5000.0f,           /* msec_t t_rot */
            .t1 = 1000.0f,           /* msec_t t1 */
            .t2 = 1000.0f,           /* msec_t t2 */
            .t3 = 1000.0f,           /* msec_t t3 */
            .dispwm = MC_FALSE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 0.0f,
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.1f,   /* mcc_t d */
        .q = 0.0f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.83f,           /*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 1600.0f,           /* vmarg_lpfcoeff */
        .spmin = 4000.0f,        /* mcsp_t spmin */
        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.00005f,             /* kp */
            .ki = 0.08f,         /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 4.5f           /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minsp = 0.0f,              /* mcsp_t minsp */
        .maxsp = 0.0f,              /* mcsp_t maxsp */
        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */
        .duration = 200u     /* mccnt_t duration */
    },
    .bratio = 52.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 100u,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -117.6548f, /* L11 */
            .ki = -18739.2f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -0.6876f,   /* L12 */
            .ki = 17.6f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 0.6239f,    /* L13 */
            .ki = -27.2f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaIq = {       /* mcpPiController_t edeltaIq */
            .kp = -117.3253f, /* L14 */
            .ki = -18739.2f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 210.0f,         /* mcsp_t minsp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.04f,          /* mcv_t k2 */
        .smc = 1600.0f,           /* mcv_t smc */
        .k1min = 2000.0f,        /* mcv_t k1min */
        .k1max = 75000.0f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_FALSE,    /* mcstate_t enabled */
        .minsp = 1800.0f,          /* mcsp_t minsp */
        .maxsp = 5250.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_FALSE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 0.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 0.0f,
    .pos_begin = 0.0f,
    .pos_mid = 0.0f,
    .pos_end = 0.0f,
    },
};
#endif
#ifdef ENABLE_WM_ORCA_FREE
const mcCp_t mcp_wm_orca_free = {    /* WM orca motor drive parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 4u,      /* mc_uint8_t pp */
        .r = 6.5f,   /* mcr_t r */
        .Ld = 0.018f,  /* mcmp_t Ld */
        .Lq = 0.018f,  /* mcmp_t Lq */
        .Ke = 0.1f,  /* mcmp_t Ke */
    },
    .freqFL = 16000u,              /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 18000.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 210.0f,            /* mcsp_t minsp_o2c */
	.minsp_c2o = 210.0f,		  	/* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d =4.5f,           /* idrefmax */
        .q = 5.5f            /* iqrefmax */
    },    
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 200.0f,         /* vdrefmax */
        .q = 200.0f          /* vqrefmax */
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 300.0f,       /* rampmaxAcc */
        .rampminAcc = 75.0f,        /* rampminAcc */
        .rampmaxDec = 300.0f,       /* rampmaxDec */
        .rampminDec = 75.0f         /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.0006314f,      /* kp */
        .ki = 0.0000235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.0006314f,      /* kp */
        .ki = 0.0000235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
        .kp = 0.0006314f,      /* kp */
        .ki = 0.0000235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 5.0f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 1800.0f,   /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 30000.0f,  /* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
        .kp = 6.957f,         /* kp */
        .ki = 1968.0f,         /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
        .kp = 10.654f,           /* kp */
        .ki = 2560.0f,          /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 200.0f           /* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_VCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
        .deadtime = 0.00792f,               /* duty_t deadtime */
        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 2.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */    
                .d = 2.5f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */     
                .d = 3.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 500.0f,        /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */   
                .d = 1.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */     
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
                .d = 1.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */     
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 0.0f,                      /* d current */
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    		            /* d current ramp */
                .q = 0.0f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idramp = 0.0008f,       /*rmpc_t idramp */
            .iqramp = 0.0f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 3.0f,   /* idmin */
                .q = 0.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 1.0f,   /* idmax */
                .q = 0.0f    /* iqmax */
            },
            .sprefmax = 6000.0f,         /* mcsp_t sprefmax */
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
            .minValidSp = 210.0f,     /* mcsp_t minValidSp */
            .tValidSp = (htck_t)250.0f,       /* msec_t tValidSp */
            .tmax = 3.0f        /* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
        .duration = 0.1f,           /* sec_t duration */
        .thErrMin = -360.0f,        /* mcdeg_t thErrMin */
        .thErrMax = 360.0f          /* mcdeg_t thErrMax */
    },
    .disPwmWhenStop = MC_FALSE,
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.5f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */      
            },
            .idq1 = {   /* mcdq_t idq1 */
                0.4f,   /* mcc_t d */
                0.0f,   /* mcc_t q */     
            },
            .idq2 = {   /* mcdq_t idq2 */
                0.3f,   /* mcc_t d */
                0.0f,   /* mcc_t q */      
            },
            .idq3 = {   /* mcdq_t idq3 */
                0.2f,   /* mcc_t d */  
                0.0f,   /* mcc_t q */        
            },
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .t_rot = 5000.0f,           /* msec_t t_rot */
            .t1 = 1000.0f,           /* msec_t t1 */
            .t2 = 1000.0f,           /* msec_t t2 */
            .t3 = 1000.0f,           /* msec_t t3 */
            .dispwm = MC_FALSE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 0.0f,
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.1f,   /* mcc_t d */
        .q = 0.0f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.83f,           /*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 1600.0f,           /* vmarg_lpfcoeff */
        .spmin = 4000.0f,        /* mcsp_t spmin */
        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.00005f,             /* kp */
            .ki = 0.08f,         /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 4.5f           /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minsp = 0.0f,              /* mcsp_t minsp */
        .maxsp = 0.0f,              /* mcsp_t maxsp */
        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */ 
        .duration = 200u     /* mccnt_t duration */            
    },
    .bratio = 15.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 100u,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -117.6548f, /* L11 */
            .ki = -18739.2f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -0.6876f,   /* L12 */
            .ki = 17.6f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 0.6239f,    /* L13 */
            .ki = -27.2f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaIq = {       /* mcpPiController_t edeltaIq */
            .kp = -117.3253f, /* L14 */
            .ki = -18739.2f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 0.1f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 210.0f,         /* mcsp_t minsp */
        .tsamp = 0.001f,         /* period_t tsamp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.04f,          /* mcv_t k2 */
        .smc = 0.1f,           /* mcv_t smc */
        .dt = 0.0000625f,     /* sec_t dt */
        .k1min = 2000.0f,        /* mcv_t k1min */
        .k1max = 75000.0f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_TRUE,    /* mcstate_t enabled */
        .minsp = 1800.0f,          /* mcsp_t minsp */
        .maxsp = 5250.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_FALSE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 0.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 0.0f,
    .pos_begin = 0.0f,
    .pos_mid = 0.0f,
    .pos_end = 0.0f,
    },
};
#endif
#ifdef ENABLE_RAC_FAN
const mcCp_t mcp_rac_fan = {    /* RAC fan motor drive parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 4u,        /* mc_uint8_t pp */
        .r = 35.25f,     /* mcr_t r */ // 70.5
        .Ld = 0.25f,    /* mcmp_t Ld */ // 0.225 (Two phase inductance) = 0.225 / 2 = 0.1125 0.250f
        .Lq = 0.25f,    /* mcmp_t Lq */ // 0.285 (Two phase inductance) = 0.285 / 2 = 0.1425 0.250f
        .Ke = 0.29f,    /* mcmp_t Ke */ //86 V/kRPM in seconds = 86 / 60 = 1.43333  1.91f
    },
    .freqFL = 8000u,              /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 900.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 300.0f,            /* mcsp_t minsp_o2c */
	.minsp_c2o = 300.0f,		  	/* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d = 0.5f,           /* idrefmax */ 
        .q = 1.0f            /* iqrefmax */
    },    
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 130.0f,         /* vdrefmax */
        .q = 130.0f          /* vqrefmax */
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 50.0f,        /* rampmaxAcc */
        .rampminAcc = 15.0f,        /* rampminAcc */
        .rampmaxDec = 50.0f,        /* rampmaxDec */
        .rampminDec = 15.0f         /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.004389f,    /* kp */
        .ki = 0.00961f,       /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 1.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.02703072f,    /* kp */
        .ki = 0.3008f,       /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 0.5f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
        .kp = 0.02703072f,    /* kp */
        .ki = 0.03008f,       /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 0.5f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 2250,      /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
        .kp = 690.57f,         /* kp */
        .ki = 9840.0f,         /* ki */
        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
        .antiWindupLimit = 100.0f           /* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
        .kp = 1060.54f,//,201.308f           /* kp */
        .ki = 12800.0f,//102400.0f,          /* ki */
        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
        .antiWindupLimit = 600.0f           /* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 80.0f,         /* d component */
        .q = 80.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
        .d = 8000.0f,         /* d component */
        .q = 8000.0f,         /* q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
        .deadtime = 0.00000f,               /* duty_t deadtime */
        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */      
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */      
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 0.0f,        /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 0.1f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */      
                .d = 0.2f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */     
                .d = 0.001f,    /* d current ramp */
                .q = 0.000f,    /* q current ramp */
            },
            .mecalign1st_duration = 500.0f,       /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */      
                .d = 0.25f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */     
                .d = 0.001f,    /* d current ramp */
                .q = 0.000f,    /* q current ramp */
            },
            .mecalign2nd_duration = 3000.0f,       /* msec_t mecalign2nd_duration */
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 0.0f,                      /* d current */
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    		            /* d current ramp */
                .q = 0.0f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idramp = 0.0001f,       /*rmpc_t idramp */
            .iqramp = 0.0000f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 0.25f,   /* idmin */
                .q = 0.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 0.4f,   /* idmax */
                .q = 0.0f    /* iqmax */
            },
            .sprefmax = 1000.0f,         /* mcsp_t sprefmax */
            .spreframp = 0.050f,          /* rmps_t spreframp */
            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
            .minValidSp = 300.0f,     /* mcsp_t minValidSp */
            .tValidSp = (htck_t)1000,       /* msec_t tValidSp */
            .tmax = 3.0f        /* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_ANGLE_SWITCHING,        /* tmode_t mode */
        .duration = 6.0f,           /* sec_t duration */
        .thErrMin = 0.0f,           /* mcdeg_t thErrMin */
        .thErrMax = 80.0f           /* mcdeg_t thErrMax */
    },
    .disPwmWhenStop = MC_TRUE,
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_DISABLE_PWM,       /* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.2f,   /* mcc_t d */  
                .q = 0.0f,   /* mcc_t q */        
            },
            .idq1 = {   /* mcdq_t idq1 */
                .d = 0.15f,   /* mcc_t d */  
                .q = 0.0f,   /* mcc_t q */        
            },
            .idq2 = {   /* mcdq_t idq2 */
                .d = 0.125f,   /* mcc_t d */  
                .q = 0.0f,   /* mcc_t q */        
            },
            .idq3 = {   /* mcdq_t idq3 */
                .d = 0.1f,   /* mcc_t d */  
                .q = 0.0f,   /* mcc_t q */        
            },
            .spreframp = 0.025f,          /* rmps_t spreframp */
            .t_rot = 10000.0f,           /* msec_t t_rot */
            .t1 = 5000.0f,           /* msec_t t1 */
            .t2 = 1000.0f,           /* msec_t t2 */
            .t3 = 1000.0f,           /* msec_t t3 */
            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 20000.0f,
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.1f,   /* mcc_t d */
        .q = 0.0f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 6400.0f,         /* mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 8.0f,           /* mcv_t vmarg_lpfcoeff */
        .spmin = 4000.0f,        /* mcsp_t spmin */
        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.00005f,             /* kp */
            .ki = 0.04f,         /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 3.0f           /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minsp = 0.0f,              /* mcsp_t minsp */
        .maxsp = 0.0f,              /* mcsp_t maxsp */
        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */ 
        .duration = 200u     /* mccnt_t duration */            
    },
    .bratio = 1.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 5000u,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -4706.192f, /* L11 */
            .ki = -1499136.0f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -27.504f,   /* L12 */
            .ki = 1408.0f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 24.956f,    /* L13 */
            .ki = -2176.0f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        {       /* mcpPiController_t edeltaIq */
            .kp = -4693.012f, /* L14 */
            .ki = -1499136.0f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 3200.0f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 210.0f,         /* mcsp_t minsp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.04f,          /* mcv_t k2 */
        .smc = 800.0f,           /* mcv_t smc */
        .k1min = 2000.0f,        /* mcv_t k1min */
        .k1max = 2062.1469f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 7840.0f,//15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_FALSE,    /* mcstate_t enabled */
        .minsp = 1800.0f,          /* mcsp_t minsp */
        .maxsp = 5250.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_TRUE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 10.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.001f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 30.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 3.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 0.0f,
    .pos_begin = 0.0f,
    .pos_mid = 0.0f,
    .pos_end = 0.0f,
    },
};
#endif
#ifdef ENABLE_RAC_COMPRESSOR
const mcCp_t mcp_rac_compressor = {    /* RAC compressor motor drive parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 3u,      /* mc_uint8_t pp */
        .r = 0.64f,   /* mcr_t r */
        .Ld = 0.00597f,  /* mcmp_t Ld */
        .Lq = 0.00853f,  /* mcmp_t Lq */
        .Ke = 0.1087f,  /* mcmp_t Ke */
    },
    .freqFL = 8000u,              /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 5000.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 200.0f,            /* mcsp_t minsp_o2c */
	.minsp_c2o = 200.0f,		    /* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d = 10.0f,           /* idrefmax */
        .q = 15.0f            /* iqrefmax */
    },    
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 300.0f,         /* vdrefmax */
        .q = 300.0f          /* vqrefmax */
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 300.0f,       /* rampmaxAcc */
        .rampminAcc = 5.0f,         /* rampminAcc */
        .rampmaxDec = 100.0f,       /* rampmaxDec */
        .rampminDec = 5.0f          /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.010466f,    /* kp */
        .ki = 0.0000197191f,       /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 13.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.010466f,    /* kp */
        .ki = 0.00197191f,       /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 13.0f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
        .kp = 0.011096f,    /* kp */
        .ki = 0.0197191f,       /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 15.0f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 1000,      /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 2000,      /* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
        .kp = 20.5f,         /* kp */
        .ki = 102.4f,         /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 380.0f           /* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
        .kp = 20.654f,           /* kp */
        .ki = 101.4f,          /* ki */
        .antiWindupCoef = 0.1f,           /* antiWindupCoef */
        .antiWindupLimit = 380.0f           /* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 8000.0f,         /* d component */
        .q = 8000.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = { /*mcmp_t inductanceCorrectionCoeff */
        .d = 8000.0f, /*d component */
        .q = 8000.0f, /*q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
        .deadtime = 0.048f,               /* duty_t deadtime */
        .dtCompType = DTCOMP_UNIDIRECTIONAL_NEG,    /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.1f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 2.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */      
                .d = 2.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */      
                .d = 3.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 500,            /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */     
                .d = 0.0f,    /* d current ramp */
                .q = 0.0f,    /* q current ramp */
            },
            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */     
                .d = 0.000f,    /* d current ramp */
                .q = 0.000f,    /* q current ramp */
            },
            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 0.0f,                      /* d current */
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.0f,    		            /* d current ramp */
                .q = 0.0f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
            .idramp = 0.01f,       /*rmpc_t idramp */
            .iqramp = 0.0f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 3.0f,   /* idmin */
                .q = 0.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 3.0f,   /* idmax */
                .q = 0.0f    /* iqmax */
            },
            .sprefmax = 1000.0f,         /* mcsp_t sprefmax */
            .spreframp = 0.300f,          /* rmps_t spreframp */
            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
            .minValidSp = 210.0f,     /* mcsp_t minValidSp */
            .tValidSp = (htck_t)1000.0f,       /* msec_t tValidSp */
            .tmax = 3.0f        /* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
        .duration = 10.0f,          /* sec_t duration */
        .thErrMin = 62.0f,          /* mcdeg_t thErrMin */
        .thErrMax = 80.0f           /* mcdeg_t thErrMax */
    },
    .disPwmWhenStop = MC_TRUE,
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_DISABLE_PWM,       /* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq1 = {   /* mcdq_t idq1 */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq2 = {   /* mcdq_t idq2 */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq3 = {   /* mcdq_t idq3 */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .spreframp = 0.050f,          /* rmps_t spreframp */
            .t_rot = 5000.0f,           /* msec_t t_rot */
            .t1 = 3000.0f,           /* msec_t t1 */
            .t2 = 2000.0f,           /* msec_t t2 */
            .t3 = 1000.0f,           /* msec_t t3 */
            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 25000.0f,
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.0f,   /* mcc_t d */
        .q = 0.0f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.6f,           /*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 80.0f,       /*mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 8.0f,          /*vmarg_lpfcoeff */
        .spmin = 2300.0f,        /* mcsp_t spmin */
        .sp_reset = 1000.0f,    /* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.00015f,             /* kp */
            .ki = 0.8f,         /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 10.0f           /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_FALSE,        /* mc_bool_t enabled */
        .minsp = 1000.0f,              /* mcsp_t minsp */
        .maxsp = 5000.0f,              /* mcsp_t maxsp */
        .lpfCoeff = 0.000125f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */ 
        .duration = 10u     /* mccnt_t duration */            
    },
    .bratio = 1.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 5000u,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -40.6548f, /* L11 */
            .ki = -1684.8f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -0.2876f,   /* L12 */
            .ki = 1.4f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 0.2239f,    /* L13 */
            .ki = -2.3f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaIq = {       /* mcpPiController_t edeltaIq */
            .kp = -40.3253f, /* L14 */
            .ki = -1684.8f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 200.0f,         /* mcsp_t minsp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.04f,          /* mcv_t k2 */
        .smc = 1600.0f,           /* mcv_t smc */
        .k1min = 2000.0f,        /* mcv_t k1min */
        .k1max = 75000.0f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 10.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_FALSE,    /* mcstate_t enabled */
        .minsp = 1800.0f,          /* mcsp_t minsp */
        .maxsp = 5250.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_FALSE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 0.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 1.0f,
    .pos_begin = 900.0f,
    .pos_mid = 150.0f,
    .pos_end = 450.0f,
    },
};
#endif
#ifdef ENABLE_REF_COMPRESSOR
const mcCp_t mcp_ref_compressor = {    /* Refrigerator compressor motor drive parameters */
    .phys = {   /* mcMotorPhysParams_t phys */
        .pp = 3u,      /* mc_uint8_t pp */
        .r =  16.600f,	//15.400f,	//3.67f,   					/* mcr_t r */
        .Ld = 0.120f,	//0.110f,	//0.165f,	/*0.070f,*/  	/* mcmp_t Ld */
        .Lq = 0.240f,	//0.220f,	//0.165f,	/*0.070f,*/  	/* mcmp_t Lq */
        .Ke = 0.210f, 	//0.206f,  	//0.215f,	/*0.094f,*/  	/* mcmp_t Ke */
    },
    .freqFL = 5500u,        /* freq_t freqFL */
    .freqSL = 1000u,               /* freq_t freqSL */
    .sprefmax = 4500.0f,           /* mcsp_t sprefmax */
    .minsp_o2c = 100.0f,            /* mcsp_t minsp_o2c */
	.minsp_c2o = 900.0f,	        /* mcsp_t minsp_c2o */
    .idqrefmax = {   /* mcdq_t idqrefmax */
        .d = 3.0f,           /* idrefmax */
        .q = 3.0f            /* iqrefmax */
    },    
    .vdqmax = {   /* mcdq_t vdqmax */
        .d = 250.0f,        /* vdrefmax */
        .q = 400.0f,       /* vqrefmax */ //250.0f,
    },
    .rampParams = {   /* mcRmpCntParams_t rampParams */
        .rampmaxAcc = 3000.0f,       /* rampmaxAcc */
        .rampminAcc = 20.0f,        /* rampminAcc */
        .rampmaxDec = 5.0f,       /* rampmaxDec */
        .rampminDec = 5.0f         /* rampminDec */
    },
    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
        .kp = 0.082f,//0.082f,//0.0205f,//0.0205f,//0.082f, //0.0082f,	//0.005f,	//0.082f,	///0.0082f,		///0.082f,	///0.082f,	//0.0075,	//7.5,			//0.00082f,	//0.0075f,	//0.00052f, 	//0.00032.f //0.0000012f,//0.00020375f,	//0.0004075f,	//0.000815f,	//0.0016314f, 	//0.0032f, 		//0.0016314f,	/* kp */
        .ki = 0.0703f,//0.0703f,//0.016f, //0.0044f,//0.016f,//0.0703f,//0.00703f,	//0.01f,	//0.0703f,	///0.000703f,	///0.07f,	///0.07f,	//0.0038,	//0.0000703f,	//0.0038f,	//0.0003f, 	//0.00000875f,	//0.000034f,	//0.00055f,		//0.0014f, 		//0.00055f,		//0.00055f,     /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
        .kp = 0.082f,///0.052f,//0.00075,//0.00062f,//0.0004078f,//0.00020375f, //0.0004078f,	//0.0004078f,	//0.0008157f,	//0.0016314f,	//0.002815f,	//0.0016314f, 	//0.0032f, 	//0.0056314f,     /* kp */
        .ki = 0.000753f,///0.0703f,//0.00070f,//0.00056f,//0.0000937f,//0.0000937f,	//0.00000875f,//0.000035f,	//0.0000937f,	//0.000375f,	//0.0014f,		//0.005875f,	//0.00055f, 	//0.0235f,	//0.0235f,        /* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
		.kp = 0.045f,//0.052f, ///0.82//0.00091,//0.00075f,//0.003600f,///0.0056314f,//0.0056314f,//0.0056314f,//0.0056314f,	/* kp */
		.ki = 0.035f,//0.07f, ///0.0005231,//0.00038f,//0.00235f,// 0.0035f//0.0235f,//0.0235f,     					/* ki */
        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
        .antiWindupLimit = 3.0f            /* antiWindupLimit */
    },
    .piSpeedR1toR2 = 1400,//1200,//1100,//1500,//1000,//2100,   /* mcsp_t piSpeedR1toR2 */
    .piSpeedR2toR3 = 3500,//4600,//4000//3500,//1700, //1900 //1800//4500, 			/* mcsp_t piSpeedR2toR3 */
    .piDCurrent = {   /* mcpPiController_t piDCurrent */
		.kp = 60.0f,//36.0f,///100.0f,//36.0f,///36.0f,///36.0f,//78.0f,//20.0f,//0.7f,//250.0f,	//12.3f,/*6.15f,*/         /* kp */
		.ki = 48.0f,//180.0f,///180.0f,///180.0f,///180.0f//3000.0f,//140.0f,//228,//2400.0f,//1150.0f,//141.08f,/*35.27f,*/ /* ki */
        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
		.antiWindupLimit = 400.0f           					/* antiWindupLimit */
    },
    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		.kp = 60.0f,///36.0f,//36.0f,//36.0f,///36.0f,//78.0f,//20.0f,//0.7,//250.0f,//340//40.0f,//250//350 //340.0f,	//15.375f, //12.3f, //6.15f,           /* kp */
		.ki = 48.0f,///180.0f,///180.0f,///180.0f ///3000.0f,//140.0f,//228,//600.0f,//500.0f,//1100//30000.0f,	//400//1150//220.437f, //141.08f, //35.27f,          /* ki */
		.antiWindupCoef = 0.01f, //0.01          				/* antiWindupCoef */
		.antiWindupLimit = 400.0f// 300.0f 150.0f           	/* antiWindupLimit */
    },
    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
    .decouplingMethod = MC_DECOUPLING_METHOD_NCALC,     /* mcDecouplingMethod_t decouplingMethod */
    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
        .d = 16000.0f,         /* d component */
        .q = 16000.0f,         /* q component */
    },
    .inductanceCorrectionCoeff = { /*mcmp_t inductanceCorrectionCoeff */
        .d = 20000.0f, /*d component */
        .q = 20000.0f, /*q component */
    },
    .modulation = {
        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
        .typeR1 = MC_MODULATION_TYPE_SVPMW_U0,  /* mc_modulation_type_t typeR1 */
        .deadtime = 0.0035f,                /* duty_t deadtime */
        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
    },
    .startup = {           /* mcsupp_t startup */
        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
        .ol = {       /* mcolp_t ol */
            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */    
                .d = 0.8f,   					/* d current */ //1.5
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */      
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */      
                .d = 0.0f,   /* d current */
                .q = 0.0f,    /* q current */
            },
            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
            .thalign_duration = 0.0f,        /* msec_t thalign_duration */
            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
            .thref_mecalign = 30.0f,     /* mcdeg_t thref_mecalign */
            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
                .d = 0.4f,///0.5///0.8f,   					/* d current */ //1.5
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */     
                .d = 0.004f,///0.005 0.008f,    				/* d current ramp */ //0.015
                .q = 0.00f,    /* q current ramp */
            },
            .mecalign1st_duration = 1000.0f, ///350     /* msec_t mecalign1st_duration */
            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
                .d = 0.8f, ///1.2f  					/* d current */
                .q = 0.0f,    /* q current */
            },
            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */     
                .d = 0.004f,///0.007f,    					/* d current ramp */
                .q = 0.00f,    /* q current ramp */
            },
            .mecalign2nd_duration = 500.0f,///200.0f,       /* msec_t mecalign2nd_duration */ //0
            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
                .d = 1.2f,///0.8f,   					/* d current */ //1.5
                .q = 0.0f,    					/* q current */
            },
            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
                .d = 0.004f,///0.008f,    				/* d current ramp */ //0.015
                .q = 0.00f,    					/* q current ramp */
            },
            .mecalign3rd_duration = 1000.0f, ///350     /* msec_t mecalign1st_duration */
            .idramp = 0.0f,       /*rmpc_t idramp */
            .iqramp = 0.0f,     /*rmpc_t iqramp */
            .idqmin = {   /* mcdq_t idqmin */
                .d = 1.2f,///1.5f   					/* idmin */
                .q = 0.0f    /* iqmin */
            },
            .idqmax = {   /* mcdq_t idqmax */
                .d = 1.2f,   					/* idmax */
                .q = 0.0f    /* iqmax */
            },
            .sprefmax = 600.0f,         		/* mcsp_t sprefmax */
            .spreframp = 0.200f,	///0.200f,//0.300f,       /* rmps_t spreframp */
            .spgapmax = 100000.0f,           /* mcsp_t spgapmax */
            .minValidSp = 599.0f,	///599.0f,///299.0f,     			/* mcsp_t minValidSp */
            .tValidSp = (htck_t)1.0f,       /* msec_t tValidSp */
            .tmax = 3.0f,///3.0f        				/* msec_t tmax */
        },
    },
    .transition = {   /* mctransp_t transition */
        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
        .duration = 1.0f,           /* sec_t duration */
        .thErrMin = 0.0f,          				/* mcdeg_t thErrMin */
        .thErrMax = 360.0f          /* mcdeg_t thErrMax */
    },
	.disPwmWhenStop = MC_FALSE,//MC_TRUE (Direct PWMdisable),
    .turnoff = {   /* mcturnoffp_t turnoff */
        .method = METHOD_DISABLE_PWM,       	/* lmethod_t method */
        .ol = {   /* mcturnoff_olp_t ol */
            .idq_rot = {   /* mcdq_t idq_rot */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq1 = {   /* mcdq_t idq1 */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq2 = {   /* mcdq_t idq2 */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .idq3 = {   /* mcdq_t idq3 */
                .d = 0.0f,   /* mcc_t d */
                .q = 0.0f,   /* mcc_t q */
            },
            .spreframp = 0.050f,          /* rmps_t spreframp */
            .t_rot = 5000.0f,           /* msec_t t_rot */
            .t1 = 10.0f,      /* msec_t t1 */
            .t2 = 10.0f,      /* msec_t t2 */
            .t3 = 10.0f,      /* msec_t t3 */
            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
        },
	.disPwmLatency = 0.0f,//2500.0f, //???
    },
    .stopped_current = {   /* mcdq_t stopped_current */
        .d = 0.0f,   /* mcc_t d */
        .q = 0.0f,   /* mcc_t q */
    },
    .fwp = {   /* mcfwp_t fwp */
        .vmaxcoeff = 0.93f,///0.92f,//0.9f,/*mcv_t vmaxcoeff */
        .vamp_lpfcoeff = 0.6f,///0.48f,///0.6f,//6.0f,//4800.0f,       	/*mcv_t vamp_lpfcoeff */
        .vmarg_lpfcoeff = 0.06f,///0.048f,//0.06f, 0.6f,          		/*vmarg_lpfcoeff */
        .spmin = 2100.0f,//2100.0f,///2000.0f,        		/* mcsp_t spmin */
        .sp_reset = 1200.0f,//1800.0f,//1500.0f,    				/* mcsp_t sp_reset */
        .pi = {   /* mcpPiController_t pi */
            .kp = 0.000414f,///0.214f,///0.414f,///0.0642f,///0.642f,///0.107f,///0.428f,///0.428f,///0.214f, /* kp */
            .ki = 0.00006f,///0.042f,///0.003f,///0.003f,///0.003f,///0.012f,///0.012f, ///0.012f, /* ki */
            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
            .antiWindupLimit = 20.0f	 	 	 	 	 	 	 	 	 /* antiWindupLimit */
        },
    },
    .mtpa = {   /* mcpMtpa_t mtpa */
        .enabled = MC_TRUE,        /* mc_bool_t enabled */
        .minsp = 950.0f,            /* mcsp_t minsp */
        .maxsp = 2100.0f,           /* mcsp_t maxsp */
        .lpfCoeff = 0.0083f,           /* mcc_t lpfCoeff */
    },
    .btstrp = {   /* mcbstrpp_t btstrp */ 
        .duration = 200u     /* mccnt_t duration */            
    },
    .bratio = 1.0f,              /* ratio_t bratio */
    .just_powered_min_duration = 100u,               /* mccnt_t just_powered_min_duration */
    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
    .rbemf = {   /* mcp_rbemf_t rbemf */
        .egammaId = {       /* mcpPiController_t egammaId */
            .kp = -480.0f, /* L11 */
            .ki = -19.2f,   /* L21 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .egammaIq = {       /* mcpPiController_t egammaIq */
            .kp = -2.8f,   /* L12 */
            .ki = 0.016f,    /* L22 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaId = {       /* mcpPiController_t edeltaId */
            .kp = 2.4f,    /* L13 */
            .ki = -0.032f,   /* L23 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .edeltaIq = {       /* mcpPiController_t edeltaIq */
            .kp = -480.0f, /* L14 */
            .ki = -19.2f,   /* L24 */
            .antiWindupCoef = 0.5f,
            .antiWindupLimit = 1000.0f,
        },
        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
        .minsp = 100.0f,         /* mcsp_t minsp */
    },
    .ato = {   /* mcp_ato_t ato */
        .k2 = 0.08f,          /* mcv_t k2 */
        .smc = 1600.0f,           /* mcv_t smc */
        .k1min = 1000.0f,        /* mcv_t k1min */
        .k1max = 1000.0f,       /* mcv_t k1max */
        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
        .thrad_lpfcoeff = 15680.0f,       /* mcrad_t thrad_lpfcoeff */
    },
    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
        .enabled = MC_FALSE,    /* mcstate_t enabled */
        .minsp = 1800.0f,          /* mcsp_t minsp */
        .maxsp = 5250.0f,          /* mcsp_t maxsp */
    },
    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
        .enabled = MC_FALSE,            /* mcstate_t enabled */
        .vabFirst = {                   /* mcab_t vabFirst */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabRamp = {                    /* mcab_t vabRam */
            .a = 0.0f,
            .b = 0.0f,
        },
        .vabMax = {                     /* mcab_t vabMax */
            .a = 0.0f, 
            .b = 0.0f,
        },
        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
    },
    .torqComp = {
    .enabled = MC_FALSE,
    .amp = 0.0f,
    .pos_begin = 0.0f,
    .pos_mid = 0.0f,
    .pos_end = 0.0f,
    },
};
#endif

//W11404160
#ifdef ENABLE_DW_CIRC_W11404160
const mcCp_t mcp_dw_circ = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 3u,      /* mc_uint8_t pp */
	        .r = 44.0f,//24.8f,   /* mcr_t r */
	        .Ld = 0.125f,//0.0865f,  /* mcmp_t Ld */
	        .Lq = 0.125f,//0.0865f,  /* mcmp_t Lq */
	        .Ke = 0.14589f,  /* mcmp_t Ke */
			.Phi = 0.17f, /* mcmp_t Phi */ /* Used in EKF Observer */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3200.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 210.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.8f,           /* idrefmax */
	        .q = 1.0f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 150.0f, //200.0f        /* vdrefmax */
	        .q = 150.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 750.0f,       /* rampmaxAcc */
	        .rampminAcc = 125.0f,        /* rampminAcc */
	        .rampmaxDec = 5000.0f,       /* rampmaxDec */
	        .rampminDec = 125.0f         /* rampminDec */
	    },
	    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
	        .kp = 0.015f, //0.0056314f,     /* kp */
	        .ki = 0.01046f, //0.0235f,       /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 1.0f            /* antiWindupLimit */
	    },
	    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
	        .kp = 0.01f, //0.0056314f,     /* kp */
	        .ki = 0.05646f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 1.0f            /* antiWindupLimit */
	    },
	    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
	        .kp = 0.01f, //0.0056314f,      /* kp */
	        .ki = 0.05646f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 1.0f            /* antiWindupLimit */
	    },
	    .piSpeedR1toR2 = 4000,      /* mcsp_t piSpeedR1toR2 */
	    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 1.5f,         /* kp */
	        .ki = 2.72f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 1.5f,         /* kp */
		    .ki = 2.72f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_VCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 76.0f,         /* d component */
	        .q = 76.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.1f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.15f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 1000,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.00005f,       /*rmpc_t idramp */
	            .iqramp = 0.0f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.01f,   /* idmin */
	                .q = 0.0f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.05f,   /* idmax */
	                .q = 0.0f    /* iqmax */
	            },
	            .sprefmax = 1000.0f,         /* mcsp_t sprefmax */
	            .spreframp = 0.3f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 50.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)1000,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_DIRECT,         /* tmode_t mode */
	        .duration = 3.0f,           /* sec_t duration */
	        .thErrMin = -40.0f, //-360.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 40.0f   //360.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 0.300f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 3000.0f,           /* msec_t t1 */
	            .t2 = 2000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -117.6548f, /* L11 */
	            .ki = -18739.2f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = -0.6876f,   /* L12 */
	            .ki = 17.6f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.6239f,    /* L13 */
	            .ki = -27.2f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaIq = {       /* mcpPiController_t edeltaIq */
	            .kp = -117.3253f, /* L14 */
	            .ki = -18739.2f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.01f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 20.0f,   	/* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.1f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      	/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, 	/* Sampling count for speed calculation */
			.speed_corr_EKF = 57.3896f,   	/* Speed correction coefficient */
			.fcutSpeed = 2000,		/* Speed LPF cut-off frequency */
			.fcutTheta = 8000,		/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
	        .k2 = 0.04f,          /* mcv_t k2 */
	        .smc = 1000.0f,           /* mcv_t smc */
	        .k1min = 2000.0f,  //500.0f      /* mcv_t k1min */
	        .k1max = 75000.0f,   //500.0f    /* mcv_t k1max */
	        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
	        .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};
#endif

//W20007335
#ifdef ENABLE_DW_CIRC
const mcCp_t mcp_dw_circ_W20007335 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 3u,      /* mc_uint8_t pp */
	        .r = 35.7f,   /* mcr_t r */
	        .Ld = 0.125f,  /* mcmp_t Ld */
	        .Lq = 0.125f,  /* mcmp_t Lq */
	        .Ke = 0.06731f,  /* mcmp_t Ke */
			.Phi = 0.06731f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.4826f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3350.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.2f,           /* idrefmax */
	        .q = 0.85f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 150.0f, //200.0f        /* vdrefmax */
	        .q = 150.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 750.0f,       /* rampmaxAcc */
	        .rampminAcc = 125.0f,        /* rampminAcc */
	        .rampmaxDec = 5000.0f,       /* rampmaxDec */
	        .rampminDec = 125.0f         /* rampminDec */
	    },
		.piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
			.kp = 0.001122f,//0.000924f,//0.00132f      /* kp */
			.ki = 0.004125f,//0.00495f, //0.0033f        /* ki */
			.antiWindupCoef = 1.0f,           /* antiWindupCoef */
			.antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
		    .kp = 0.0004f, //0.0056314f,     /* kp */
		    .ki = 0.0010f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
		    .kp = 0.0002f, //0.0056314f,      /* kp */
		    .ki = 0.0005f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR1toR2 = 4000,      /* mcsp_t piSpeedR1toR2 */
		.piSpeedR2toR3 = 5000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 220.1f,         /* kp */
	        .ki = 60732.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 220.1f,         /* kp */
		    .ki = 60732.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.3f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.1f,   /* idmin */
	                .q = 0.3f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.1f,   /* idmax */
	                .q = 0.6f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)600,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
	        .duration = 2.0f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-120.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //120.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -232.03f, /* L11 */
	            .ki = -197392.09f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaIq = {       /* mcpPiController_t edeltaIq */
	            .kp = -232.03f, /* L14 */
	            .ki = -197392.09f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.01f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 20.0f,   	/* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.1f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      	/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, 	/* Sampling count for speed calculation */
			.speed_corr_EKF = 57.3896f,   	/* Speed correction coefficient */
			.fcutSpeed = 2000,		/* Speed LPF cut-off frequency */
			.fcutTheta = 8000,		/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
		    .k2 = 0.04f,          /* mcv_t k2 */
		    .smc = 1600.0f,           /* mcv_t smc */
		    .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
		    .k1max = 4700.0f,   //500.0f    /* mcv_t k1max */
		    .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
		    .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};
#endif

//W11715301
#ifdef ENABLE_DW_CIRC
const mcCp_t mcp_dw_circ_W11715301 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 3u,      /* mc_uint8_t pp */
	        .r = 35.7f,   /* mcr_t r */
	        .Ld = 0.125f,  /* mcmp_t Ld */
	        .Lq = 0.125f,  /* mcmp_t Lq */
	        .Ke = 0.06731f,  /* mcmp_t Ke */
			.Phi = 0.06731f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.4826f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3350.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.2f,           /* idrefmax */
	        .q = 0.85f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 150.0f, //200.0f        /* vdrefmax */
	        .q = 150.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 750.0f,       /* rampmaxAcc */
	        .rampminAcc = 125.0f,        /* rampminAcc */
	        .rampmaxDec = 5000.0f,       /* rampmaxDec */
	        .rampminDec = 125.0f         /* rampminDec */
	    },
		.piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
			.kp = 0.001122f,//0.000924f,//0.00132f      /* kp */
			.ki = 0.004125f,//0.00495f, //0.0033f        /* ki */
			.antiWindupCoef = 1.0f,           /* antiWindupCoef */
			.antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
		    .kp = 0.0004f, //0.0056314f,     /* kp */
		    .ki = 0.0010f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
		    .kp = 0.0002f, //0.0056314f,      /* kp */
		    .ki = 0.0005f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR1toR2 = 4000,      /* mcsp_t piSpeedR1toR2 */
		.piSpeedR2toR3 = 5000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 220.1f,         /* kp */
	        .ki = 60732.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 220.1f,         /* kp */
		    .ki = 60732.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.3f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.0f,   /* idmin */
	                .q = 0.3f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.0f,   /* idmax */
	                .q = 0.85f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)600,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_DIRECT,         /* tmode_t mode */
	        .duration = 3.0f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-120.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //120.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -232.03f, /* L11 */
	            .ki = -197392.09f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaIq = {       /* mcpPiController_t edeltaIq */
	            .kp = -232.03f, /* L14 */
	            .ki = -197392.09f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.01f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 20.0f,   	/* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.1f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      	/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, 	/* Sampling count for speed calculation */
			.speed_corr_EKF = 57.3896f,   	/* Speed correction coefficient */
			.fcutSpeed = 2000,		/* Speed LPF cut-off frequency */
			.fcutTheta = 8000,		/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
		    .k2 = 0.04f,          /* mcv_t k2 */
		    .smc = 1600.0f,           /* mcv_t smc */
		    .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
		    .k1max = 4700.0f,   //500.0f    /* mcv_t k1max */
		    .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
		    .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};
#endif

//W20016965
#ifdef ENABLE_DW_CIRC
const mcCp_t mcp_dw_circ_W20016965 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 3u,      /* mc_uint8_t pp */
	        .r = 35.7f,   /* mcr_t r */
	        .Ld = 0.125f,  /* mcmp_t Ld */
	        .Lq = 0.125f,  /* mcmp_t Lq */
	        .Ke = 0.06731f,  /* mcmp_t Ke */
			.Phi = 0.06731f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.4826f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3350.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.2f,           /* idrefmax */
	        .q = 0.85f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 150.0f, //200.0f        /* vdrefmax */
	        .q = 150.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 750.0f,       /* rampmaxAcc */
	        .rampminAcc = 125.0f,        /* rampminAcc */
	        .rampmaxDec = 5000.0f,       /* rampmaxDec */
	        .rampminDec = 125.0f         /* rampminDec */
	    },
		.piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
			.kp = 0.001122f,//0.000924f,//0.00132f      /* kp */
			.ki = 0.004125f,//0.00495f, //0.0033f        /* ki */
			.antiWindupCoef = 1.0f,           /* antiWindupCoef */
			.antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
		    .kp = 0.0004f, //0.0056314f,     /* kp */
		    .ki = 0.0010f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
		    .kp = 0.0002f, //0.0056314f,      /* kp */
		    .ki = 0.0005f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR1toR2 = 4000,      /* mcsp_t piSpeedR1toR2 */
		.piSpeedR2toR3 = 5000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 220.1f,         /* kp */
	        .ki = 60732.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 220.1f,         /* kp */
		    .ki = 60732.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 150.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.3f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.0f,   /* idmin */
	                .q = 0.3f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.0f,   /* idmax */
	                .q = 0.85f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)600,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_DIRECT,         /* tmode_t mode */
	        .duration = 3.0f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-120.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //120.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -232.03f, /* L11 */
	            .ki = -197392.09f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaIq = {       /* mcpPiController_t edeltaIq */
	            .kp = -232.03f, /* L14 */
	            .ki = -197392.09f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.01f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 20.0f,   	/* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.1f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      	/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, 	/* Sampling count for speed calculation */
			.speed_corr_EKF = 57.3896f,   	/* Speed correction coefficient */
			.fcutSpeed = 2000,		/* Speed LPF cut-off frequency */
			.fcutTheta = 8000,		/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
		    .k2 = 0.04f,          /* mcv_t k2 */
		    .smc = 1600.0f,           /* mcv_t smc */
		    .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
		    .k1max = 4700.0f,   //500.0f    /* mcv_t k1max */
		    .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
		    .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};
#endif

//W11652801
#ifdef ENABLE_DW_CIRC
const mcCp_t mcp_dw_circ_W11652801 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 3u,      /* mc_uint8_t pp */
	        .r = 25.4f,   /* mcr_t r */
	        .Ld = 0.079f,  /* mcmp_t Ld */
	        .Lq = 0.079f,  /* mcmp_t Lq */
	        .Ke = 0.0753f,  /* mcmp_t Ke */
			.Phi = 0.0753f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.51f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3500.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.2f,           /* idrefmax */
	        .q = 0.85f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 150.0f, //200.0f        /* vdrefmax */
	        .q = 150.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 750.0f,       /* rampmaxAcc */
	        .rampminAcc = 125.0f,        /* rampminAcc */
	        .rampmaxDec = 4000.0f,       /* rampmaxDec */
	        .rampminDec = 125.0f         /* rampminDec */
	    },
		.piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
			.kp = 0.0005744f,//0.000924f,//0.00132f      /* kp */
			.ki = 0.002154f,//0.00495f, //0.0033f        /* ki */
			.antiWindupCoef = 1.0f,           /* antiWindupCoef */
			.antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
		    .kp = 0.0004f, //0.0056314f,     /* kp */
		    .ki = 0.0010f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
		    .kp = 0.0002f, //0.0056314f,      /* kp */
		    .ki = 0.0005f, //0.0235f,        /* ki */
		    .antiWindupCoef = 1.0f,           /* antiWindupCoef */
		    .antiWindupLimit = 0.85f            /* antiWindupLimit */
		},
		.piSpeedR1toR2 = 4000,      /* mcsp_t piSpeedR1toR2 */
		.piSpeedR2toR3 = 5000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 184.9f,         /* kp */
	        .ki = 59505.5f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 100.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 184.9f,         /* kp */
		    .ki = 59505.5f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 100.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.0f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.16f,   /* idmin */
	                .q = 0.50f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.16f,   /* idmax */
	                .q = 0.50f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 100.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)200,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
	        .duration = 5.0f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-120.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //120.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -143.3f, /* L11 */
	            .ki = -124751.8f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaIq = {       /* mcpPiController_t edeltaIq */
	            .kp = -143.3f, /* L14 */
	            .ki = -124751.8f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.01f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 20.0f,   	/* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.1f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      	/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, 	/* Sampling count for speed calculation */
			.speed_corr_EKF = 57.3896f,   	/* Speed correction coefficient */
			.fcutSpeed = 2000,		/* Speed LPF cut-off frequency */
			.fcutTheta = 8000,		/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
		    .k2 = 0.04f,          /* mcv_t k2 */
		    .smc = 1600.0f,           /* mcv_t smc */
		    .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
		    .k1max = 7403.0f,   //500.0f    /* mcv_t k1max */
		    .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
		    .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};
#endif

//W10912610
#ifdef ENABLE_DW_DRAIN_W10912610
const mcCp_t mcp_dw_drain_W10912610 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 1u,      /* mc_uint8_t pp */
	        .r = 44.0f,   /* mcr_t r */
	        .Ld = 0.14f,  /* mcmp_t Ld */
	        .Lq = 0.14f,  /* mcmp_t Lq */
	        .Ke = 0.0245f,  /* mcmp_t Ke */
			.Phi = 0.153f, /* mcmp_t Phi */ /* Used in EKF Observer */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3100.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 210.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.85f,           /* idrefmax */
	        .q = 0.85f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 60.0f, //200.0f        /* vdrefmax */
	        .q = 60.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 150.0f,       /* rampmaxAcc */
	        .rampminAcc = 100.0f,        /* rampminAcc */
	        .rampmaxDec = 5000.0f,       /* rampmaxDec */
	        .rampminDec = 100.0f         /* rampminDec */
	    },
	    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
	        .kp = 0.05f, //0.0056314f,     /* kp */
	        .ki = 0.5646f, //0.0235f,       /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.85f            /* antiWindupLimit */
	    },
	    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
	        .kp = 0.03f, //0.0056314f,     /* kp */
	        .ki = 0.015f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.85f            /* antiWindupLimit */
	    },
	    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
	        .kp = 0.03f, //0.0056314f,      /* kp */
	        .ki = 0.02645f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.85f            /* antiWindupLimit */
	    },
	    .piSpeedR1toR2 = 2500,      /* mcsp_t piSpeedR1toR2 */
	    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 0.75f,         /* kp */
	        .ki = 15.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 60.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 0.75f,         /* kp */
		    .ki = 15.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 60.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_VCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 76.0f,         /* d component */
	        .q = 76.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.225f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.25f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 1000,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.0f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.0f,   /* idmin */
	                .q = 0.0f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.0f,   /* idmax */
	                .q = 0.0f    /* iqmax */
	            },
	            .sprefmax = 1000.0f,         /* mcsp_t sprefmax */
	            .spreframp = 0.1f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 50.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)500,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_DIRECT,         /* tmode_t mode */
	        .duration = 3.0f,           /* sec_t duration */
	        .thErrMin = -100.0f, //-360.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 100.0f   //360.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_FALSE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 0.300f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 3000.0f,           /* msec_t t1 */
	            .t2 = 2000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_FALSE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -784.36f/5.0f, /* L11 */
	            .ki = -124928.0f/5.0f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = -4.58f/5.0f,   /* L12 */
	            .ki = 117.33f/5.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 4.16f/5.0f,    /* L13 */
	            .ki = -181.33f/5.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        {       /* mcpPiController_t edeltaIq */
	            .kp = -782.16f/5.0f, /* L14 */
	            .ki = -124928.0f/5.0f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.025f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 5.0f,   	    /* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.05f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      		/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, /* Sampling count for speed calculation */
			.speed_corr_EKF = 172.1688f,/* Speed correction coefficient */
			.fcutSpeed = 2000,			/* Speed LPF cut-off frequency */
			.fcutTheta = 16000,			/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
	        .k2 = 0.04f,          /* mcv_t k2 */
	        .smc = 1600.0f,           /* mcv_t smc */
	        .k1min = 2000.0f,  //500.0f      /* mcv_t k1min */
	        .k1max = 75000.0f,   //500.0f    /* mcv_t k1max */
	        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
	        .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};
#endif


//W11402566
#ifdef ENABLE_DW_DRAIN
const mcCp_t mcp_dw_drain_W11402566 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 1u,      /* mc_uint8_t pp */
	        .r = 65.0f,   /* mcr_t r */
	        .Ld = 0.13f,  /* mcmp_t Ld */
	        .Lq = 0.13f,  /* mcmp_t Lq */
	        .Ke = 0.06614f,  /* mcmp_t Ke */
			.Phi = 0.06614f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.14f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3500.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.20f,           /* idrefmax */
	        .q = 0.58f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 60.0f, //200.0f        /* vdrefmax */
	        .q = 100.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 4000.0f,       /* rampmaxAcc */
	        .rampminAcc = 100.0f,        /* rampminAcc */
	        .rampmaxDec = 4000.0f,       /* rampmaxDec */
	        .rampminDec = 100.0f         /* rampminDec */
	    },
	    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
	        .kp = 0.000924f,//0.004536f,      /* kp */
	        .ki = 0.00495f,//0.006048f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.58f            /* antiWindupLimit */
	    },
	    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
	        .kp = 0.001542f, //0.0056314f,     /* kp */
	        .ki = 0.01935f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.58f            /* antiWindupLimit */
	    },
	    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
	        .kp = 0.03f, //0.0056314f,      /* kp */
	        .ki = 0.02646f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.58f            /* antiWindupLimit */
	    },
	    .piSpeedR1toR2 = 1200,      /* mcsp_t piSpeedR1toR2 */
	    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 234.0f,//195.0f,         /* kp */
	        .ki = 169000.0f,//130000.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 60.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 234.0f,//195.0f,         /* kp */
		    .ki = 169000.0f,//130000.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 60.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.00f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.02f,   /* idmin */
	                .q = 0.2f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.02f,   /* idmax */
	                .q = 0.2f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 50.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)700,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
	        .duration = 0.2f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-180.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //180.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -212.0f, /* L11 */
	            .ki = -205238.0f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        {       /* mcpPiController_t edeltaIq */
	            .kp = -212.0f, /* L14 */
	            .ki = -205238.0f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.025f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 5.0f,   	    /* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.05f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      		/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, /* Sampling count for speed calculation */
			.speed_corr_EKF = 172.1688f,/* Speed correction coefficient */
			.fcutSpeed = 2000,			/* Speed LPF cut-off frequency */
			.fcutTheta = 16000,			/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
	        .k2 = 0.04f,          /* mcv_t k2 */
	        .smc = 1600.0f,           /* mcv_t smc */
	        .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
	        .k1max = 4700.0f,   //500.0f    /* mcv_t k1max */
	        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
	        .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};

#endif

//W11186148
#ifdef ENABLE_DW_DRAIN
const mcCp_t mcp_dw_drain_W11186148 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 1u,      /* mc_uint8_t pp */
	        .r = 45.5f,   /* mcr_t r */
	        .Ld = 0.12f,  /* mcmp_t Ld */
	        .Lq = 0.12f,  /* mcmp_t Lq */
	        .Ke = 0.07268f,  /* mcmp_t Ke */
			.Phi = 0.07268f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.15f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3500.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.20f,           /* idrefmax */
	        .q = 0.58f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 60.0f, //200.0f        /* vdrefmax */
	        .q = 100.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 4000.0f,       /* rampmaxAcc */
	        .rampminAcc = 100.0f,        /* rampminAcc */
	        .rampmaxDec = 4000.0f,       /* rampmaxDec */
	        .rampminDec = 100.0f         /* rampminDec */
	    },
	    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
	        .kp = 0.000924f,//0.004536f,      /* kp */
	        .ki = 0.00495f,//0.006048f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.58f            /* antiWindupLimit */
	    },
	    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
	        .kp = 0.001542f, //0.0056314f,     /* kp */
	        .ki = 0.01935f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.58f            /* antiWindupLimit */
	    },
	    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
	        .kp = 0.03f, //0.0056314f,      /* kp */
	        .ki = 0.02646f, //0.0235f,        /* ki */
	        .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.58f            /* antiWindupLimit */
	    },
	    .piSpeedR1toR2 = 1200,      /* mcsp_t piSpeedR1toR2 */
	    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 234.0f,//195.0f,         /* kp */
	        .ki = 169000.0f,//130000.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 60.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 234.0f,//195.0f,         /* kp */
		    .ki = 169000.0f,//130000.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 60.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.2f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.00f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.02f,   /* idmin */
	                .q = 0.2f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.02f,   /* idmax */
	                .q = 0.2f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 50.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)700,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
	        .duration = 0.2f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-180.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //180.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -212.0f, /* L11 */
	            .ki = -205238.0f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        {       /* mcpPiController_t edeltaIq */
	            .kp = -212.0f, /* L14 */
	            .ki = -205238.0f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.025f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 5.0f,   	    /* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.05f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      		/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, /* Sampling count for speed calculation */
			.speed_corr_EKF = 172.1688f,/* Speed correction coefficient */
			.fcutSpeed = 2000,			/* Speed LPF cut-off frequency */
			.fcutTheta = 16000,			/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
	        .k2 = 0.04f,          /* mcv_t k2 */
	        .smc = 1600.0f,           /* mcv_t smc */
	        .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
	        .k1max = 4700.0f,   //500.0f    /* mcv_t k1max */
	        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
	        .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};

#endif

//W11377410
#ifdef ENABLE_DW_DRAIN
const mcCp_t mcp_dw_drain_W11377410 = {    /* WM orca motor drive parameters */
	    .phys = {   /* mcMotorPhysParams_t phys */
	        .pp = 1u,      /* mc_uint8_t pp */
	        .r = 46.6f,   /* mcr_t r */
	        .Ld = 0.133f,  /* mcmp_t Ld */
	        .Lq = 0.133f,  /* mcmp_t Lq */
	        .Ke = 0.0726f,  /* mcmp_t Ke */
			.Phi = 0.0726f, /* mcmp_t Phi */ /* Used in EKF Observer */
			.Kt = 0.15f,    /* mcmp_t Kt */
	    },
	    .freqFL = 7550u,              /* freq_t freqFL */
	    .freqSL = 1000u,               /* freq_t freqSL */
	    .sprefmax = 3500.0f,           /* mcsp_t sprefmax */
	    .minsp_o2c = 210.0f,           	/* mcsp_t minsp_o2c */
		.minsp_c2o = 390.0f,		    /* mcsp_t minsp_c2o */
	    .idqrefmax = {   /* mcdq_t idqrefmax */
	        .d = 0.10f,           /* idrefmax */
	        .q = 0.46f            /* iqrefmax */
	    },
	    .vdqmax = {   /* mcdq_t vdqmax */
	        .d = 40.0f, //200.0f        /* vdrefmax */
	        .q = 100.0f  //200.0f        /* vqrefmax */
	    },
	    .rampParams = {   /* mcRmpCntParams_t rampParams */
	        .rampmaxAcc = 4000.0f,       /* rampmaxAcc */
	        .rampminAcc = 100.0f,        /* rampminAcc */
	        .rampmaxDec = 4000.0f,       /* rampmaxDec */
	        .rampminDec = 100.0f         /* rampminDec */
	    },
	    .piSpeedR1 = {   /* mcpPiController_t piSpeedR1 */
	        .kp = 0.00092f,//0.004536f,      /* kp */
	        .ki = 0.0119f,//0.006048f,        /* ki */
	        .antiWindupCoef = 0.5f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.46f            /* antiWindupLimit */
	    },
	    .piSpeedR2 = {   /* mcpPiController_t piSpeedR2 */
	        .kp = 0.002f, //0.0056314f,     /* kp */
	        .ki = 0.0119f, //0.0235f,        /* ki */
	        .antiWindupCoef = 0.5f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.46f            /* antiWindupLimit */
	    },
	    .piSpeedR3 = {   /* mcpPiController_t piSpeedR3 */
	        .kp = 0.03f, //0.0056314f,      /* kp */
	        .ki = 0.02646f, //0.0235f,        /* ki */
	        .antiWindupCoef = 0.5f,           /* antiWindupCoef */
	        .antiWindupLimit = 0.46f            /* antiWindupLimit */
	    },
	    .piSpeedR1toR2 = 4000,      /* mcsp_t piSpeedR1toR2 */
	    .piSpeedR2toR3 = 9000,      /* mcsp_t piSpeedR2toR3 */
	    .piDCurrent = {   /* mcpPiController_t piDCurrent */
	        .kp = 180.9f,//195.0f,         /* kp */
	        .ki = 63452.8f,//130000.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 20.0f  //200.0f         /* antiWindupLimit */
	    },
	    .zcCoeffDCurrent = 1.0f,            /* period_t zcCoeffDCurrent */
	    .piQCurrent = {   /* mcpPiController_t piQCurrent */
		    .kp = 180.9f,//195.0f,         /* kp */
		    .ki = 63452.8f,//130000.0f,         /* ki */
	        .antiWindupCoef = 0.01f,           /* antiWindupCoef */
	        .antiWindupLimit = 70.0f  //200.0f            /* antiWindupLimit */
	    },
	    .zcCoeffQCurrent = 1.0f,            /* period_t zcCoeffQCurrent */
	    .decouplingMethod = MC_DECOUPLING_METHOD_CCALC,     /* mcDecouplingMethod_t decouplingMethod */
	    .decoupling_lpfcoeff = {   /* mcdq_t decoupling_lpfcoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .inductanceCorrectionCoeff = {         /* mcmp_t inductanceCorrectionCoeff */
	        .d = 7600.0f,         /* d component */
	        .q = 7600.0f,         /* q component */
	    },
	    .modulation = {
	        .speedR1 = 0.0f,                        /* mcsp_t speedR1 */
	        .typeR0 = MC_MODULATION_TYPE_NONE,      /* mc_modulation_type_t typeR0 */
	        .typeR1 = MC_MODULATION_TYPE_SVPMW,     /* mc_modulation_type_t typeR1 */
	        .deadtime = 0.0f,                   /* duty_t deadtime */
	        .dtCompType = DTCOMP_BIDIRECTIONAL,     /* mcDtCompType_t dtCompType */
	        .dt_trans_duty = 0.0f,          /* duty_t  dt_trans_duty*/
	    },
	    .startup = {           /* mcsupp_t startup */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {       /* mcolp_t ol */
	            .idq_thalign_0 = {       /* mcdq_t idq_thalign_0 */
	                .d = 0.1f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_1st = {       /* mcdq_t idq_thalign_1st */
	                .d = 0.1f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idq_thalign_2nd = {       /* mcdq_t idq_thalign_2nd */
	                .d = 0.1f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .thref_thalign = 0.0f,  /* mcdeg_t thref_thalign */
	            .thalign_duration = 333,            /* msec_t thalign_duration */
	            .thermalRefCurrentLatency = 50,     /* msec_t thermalRefCurrentLatency */
	            .thref_mecalign = 0.0f,     /* mcdeg_t thref_mecalign */
	            .idq_mecalign1st = {       /* mcdq_t idq_mecalign1st */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign1st = {       /* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    /* d current ramp */
	                .q = 0.0f,    /* q current ramp */
	            },
	            .mecalign1st_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idq_mecalign2nd = {       /* mcdq_t idq_mecalign2nd */
	                .d = 0.0f,   /* d current */
	                .q = 0.0f,    /* q current */
	            },
	            .idqramp_mecalign2nd = {       /* mcdq_t idqramp_mecalign2nd */
	                .d = 0.000f,    /* d current ramp */
	                .q = 0.000f,    /* q current ramp */
	            },
	            .mecalign2nd_duration = 0.0f,       /* msec_t mecalign2nd_duration */
	            .idq_mecalign3rd = {       			/* mcdq_t idq_mecalign1st */
	                .d = 0.0f,                      /* d current */
	                .q = 0.0f,    					/* q current */
	            },
	            .idqramp_mecalign3rd = {       		/* mcdq_t idqramp_mecalign1st */
	                .d = 0.0f,    		            /* d current ramp */
	                .q = 0.0f,    					/* q current ramp */
	            },
	            .mecalign3rd_duration = 0.0f,       /* msec_t mecalign1st_duration */
	            .idramp = 0.0f,       /*rmpc_t idramp */
	            .iqramp = 0.0f,     /*rmpc_t iqramp */
	            .idqmin = {   /* mcdq_t idqmin */
	                .d = 0.064f,   /* idmin */
	                .q = 0.20f    /* iqmin */
	            },
	            .idqmax = {   /* mcdq_t idqmax */
	                .d = 0.064f,   /* idmax */
	                .q = 0.20f    /* iqmax */
	            },
	            .sprefmax = 900.0f,         /* mcsp_t sprefmax */
	            .spreframp = 4.0f, //0.3f          /* rmps_t spreframp */
	            .spgapmax = 50.0f,           /* mcsp_t spgapmax */
	            .minValidSp = 210.0f,//210.0f     /* mcsp_t minValidSp */
	            .tValidSp = (htck_t)200,       /* msec_t tValidSp */
	            .tmax = 3.0f        /* msec_t tmax */
	        },
	    },
	    .transition = {   /* mctransp_t transition */
	        .mode = TRANSITION_MODE_ANGLE_SWITCHING,         /* tmode_t mode */
	        .duration = 1.0f,           /* sec_t duration */
	        .thErrMin = -360.0f, //-360.0f       /* mcdeg_t thErrMin */
	        .thErrMax = 360.0f   //360.0f       /* mcdeg_t thErrMax */
	    },
	    .disPwmWhenStop = MC_TRUE,
	    .turnoff = {   /* mcturnoffp_t turnoff */
	        .method = METHOD_OPEN_LOOP,       /* lmethod_t method */
	        .ol = {   /* mcturnoff_olp_t ol */
	            .idq_rot = {   /* mcdq_t idq_rot */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq1 = {   /* mcdq_t idq1 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq2 = {   /* mcdq_t idq2 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .idq3 = {   /* mcdq_t idq3 */
	                .d = 0.0f,   /* mcc_t d */
	                .q = 0.0f,   /* mcc_t q */
	            },
	            .spreframp = 4.0f,          /* rmps_t spreframp */
	            .t_rot = 5000.0f,           /* msec_t t_rot */
	            .t1 = 1000.0f,           /* msec_t t1 */
	            .t2 = 1000.0f,           /* msec_t t2 */
	            .t3 = 1000.0f,           /* msec_t t3 */
	            .dispwm = MC_TRUE,      /* mc_bool_t dispwm */
	        },
		.disPwmLatency = 0.0f,
	    },
	    .stopped_current = {   /* mcdq_t stopped_current */
	        .d = 0.0f,   /* mcc_t d */
	        .q = 0.0f,   /* mcc_t q */
	    },
	    .fwp = {   /* mcfwp_t fwp */
	        .vmaxcoeff = 0.8f,           /*mcv_t vmaxcoeff */
	        .vamp_lpfcoeff = 12800.0f,         /* mcv_t vamp_lpfcoeff */
	        .vmarg_lpfcoeff = 16.0f,           /* vmarg_lpfcoeff */
	        .spmin = 5000.0f,        /* mcsp_t spmin */
	        .sp_reset = 1800.0f,    /* mcsp_t sp_reset */
	        .pi = {   /* mcpPiController_t pi */
	            .kp = 0.00005f,             /* kp */
	            .ki = 1.28f,         /* ki */
	            .antiWindupCoef = 1.0f,           /* antiWindupCoef */
	            .antiWindupLimit = 3.0f           /* antiWindupLimit */
	        },
	    },
	    .mtpa = {   /* mcpMtpa_t mtpa */
	        .enabled = MC_FALSE,        /* mc_bool_t enabled */
	        .minsp = 0.0f,              /* mcsp_t minsp */
	        .maxsp = 0.0f,              /* mcsp_t maxsp */
	        .lpfCoeff = 0.0f,           /* mcc_t lpfCoeff */
	    },
	    .btstrp = {   /* mcbstrpp_t btstrp */
	        .duration = 0 //200     /* mccnt_t duration */
	    },
	    .bratio = 1.0f,              /* ratio_t bratio */
	    .just_powered_min_duration = 0, //100              /* mccnt_t just_powered_min_duration */
	    .rampController = RAMP_CONTROLLER_RPIOBSERVER,        /* rampController_t rampController */
	    .rbemf = {   /* mcp_rbemf_t rbemf */
	        .egammaId = {       /* mcpPiController_t egammaId */
	            .kp = -237.5f, /* L11 */
	            .ki = -210025.2f,   /* L21 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .egammaIq = {       /* mcpPiController_t egammaIq */
	            .kp = 0.0f,   /* L12 */
	            .ki = 0.0f,    /* L22 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .edeltaId = {       /* mcpPiController_t edeltaId */
	            .kp = 0.0f,    /* L13 */
	            .ki = 0.0f,   /* L23 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        {       /* mcpPiController_t edeltaIq */
	            .kp = -237.5f, /* L14 */
	            .ki = -210025.2f,   /* L24 */
	            .antiWindupCoef = 0.5f,
	            .antiWindupLimit = 1000.0f,
	        },
	        .lpfcoeff = 1600.0f,           /* mcv_t lpfcoeff */
	        .egammaMax = 1000.0f,        /* mcv_t egammaMax */
	        .edeltaMax = 1000.0f,        /* mcv_t edeltaMax */
	        .edeltaMin = 0.01f,           /* mcv_t edeltaMin */
	        .minsp = 210.0f,         /* mcsp_t minsp */
	    },
		.ekf = {   /* mcp_ekf_t ekf */
		    .Qk_ol[0][0] = 0.025f,     	/* Alpha-Beta current correction coefficient at open loop*/
			.Qk_ol[3][3] = 5.0f,   	    /* Position estimation correction coefficient*/
			.Qk_cls[0][0] = 0.05f,    	/* Alpha-Beta current correction coefficient at closed loop */
			.Rk[0][0] = 1.0f,      		/* Sensor data correction coefficient */
			.theta_EKF_Shift = 0.0f,  	/* Position offset */
			.sampling_speed_EKF = 1000, /* Sampling count for speed calculation */
			.speed_corr_EKF = 172.1688f,/* Speed correction coefficient */
			.fcutSpeed = 2000,			/* Speed LPF cut-off frequency */
			.fcutTheta = 16000,			/* Theta LPF cut-off frequency */
	    },
	    .ato = {   /* mcp_ato_t ato */
	        .k2 = 0.04f,          /* mcv_t k2 */
	        .smc = 1600.0f,           /* mcv_t smc */
	        .k1min = 1000.0f,  //500.0f      /* mcv_t k1min */
	        .k1max = 4500.0f,   //500.0f    /* mcv_t k1max */
	        .k1_merge_step = 0.001f,          /* mcv_t k1_merge_step */
	        .thrad_lpfcoeff = 7840.0f,       /* mcrad_t thrad_lpfcoeff */
	    },
	    .thest_offset = 0.0f,       /* mcdeg_t thest_offset */
	    .hwHardBrake = {    /* mcp_hwHardBrake_t hwHardBrake */
	        .enabled = MC_FALSE,    /* mcstate_t enabled */
	        .minsp = 1800.0f,          /* mcsp_t minsp */
	        .maxsp = 5250.0f,          /* mcsp_t maxsp */
	    },
	    .mcOnTheFlyStartup = {              /* mcpOnTheFlyStartup_t mcOnTheFlyStartup */
	        .enabled = MC_FALSE,            /* mcstate_t enabled */
	        .vabFirst = {                   /* mcab_t vabFirst */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabRamp = {                    /* mcab_t vabRam */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .vabMax = {                     /* mcab_t vabMax */
	            .a = 0.0f,
	            .b = 0.0f,
	        },
	        .iAveIntegPerSecMin = 0.0f,     /* mcc_t iAveIntegPerSecMin */
	    },
	    .torqComp = {
	    .enabled = MC_FALSE,
	    .amp = 0.0f,
	    .pos_begin = 0.0f,
	    .pos_mid = 0.0f,
	    .pos_end = 0.0f,
	    },
};

#endif

const mcCp_t* mcp_px[SUPPORTED_TOTAL_MOTOR] = {
      #ifdef ENABLE_WM_ORCA
      &mcp_wm_orca,
      #endif
      #ifdef ENABLE_DRY_ORCA
	  &mcp_dry_orca,
      #endif
      #ifdef ENABLE_RAC_COMPRESSOR
      &mcp_rac_compressor,
      #endif
      #ifdef ENABLE_RAC_FAN
      &mcp_rac_fan,
      #endif
	  #ifdef ENABLE_DW_CIRC
	  &mcp_dw_circ_W20007335,
      #endif
	  #ifdef ENABLE_DW_DRAIN
	  &mcp_dw_drain_W11402566,
	  #endif
      #ifdef ENABLE_REF_COMPRESSOR
      &mcp_ref_compressor,
      #endif
};

/************************ Variable object definitions - stop ******************/
