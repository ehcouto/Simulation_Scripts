#ifndef FSCONSTS_H
#define FSCONSTS_H

#define LOP_MIN_CURRENT_COEFF           (0.35356f)   /* rms/2 : sqrt(2)/2 */
#define LOP_MIN_CURRENT_COEFF_DRAIN		(0.05f)

#define NO_CURRENT_POS1_PHU_MIN         (250.0f)     /* (0-90-arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS1_PHU_MAX         (290.0f)     /* (0-90+arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS2_PHU_MIN         (70.0f)      /* (0+90-arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS2_PHU_MAX         (110.0f)     /* (0+90+arcsin(LOP_MIN_CURRENT_COEFF)) */

#define NO_CURRENT_POS1_PHV_MIN         (10.0f)      /* (120-90-arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS1_PHV_MAX         (50.0f)      /* (120-90+arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS2_PHV_MIN         (190.0f)     /* (120+90-arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS2_PHV_MAX         (230.0f)     /* (120+90+arcsin(LOP_MIN_CURRENT_COEFF)) */

#define NO_CURRENT_POS1_PHW_MIN         (130.0f)     /* (240-90-arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS1_PHW_MAX         (170.0f)     /* (240-90+arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS2_PHW_MIN         (310.0f)     /* (240+90-arcsin(LOP_MIN_CURRENT_COEFF)) */
#define NO_CURRENT_POS2_PHW_MAX         (350.0f)     /* (240+90+arcsin(LOP_MIN_CURRENT_COEFF)) */

#define FS_BINARY_FLAG_DCBUS                    (0x00000001)    /* Latest generated fault memory variable flag; dc bus fault */
#define FS_BINARY_FLAG_IPM_FAULT                (0x00000002)    /* Latest generated fault memory variable flag; ipm fault */
#define FS_BINARY_FLAG_OVER_CURRENT             (0x00000004)    /* Latest generated fault memory variable flag; over current fault */
#define FS_BINARY_FLAG_CURRENT_SAMPLING         (0x00000008)    /* Latest generated fault memory variable flag; current sampling fault */
#define FS_BINARY_FLAG_TARGET_CURRENT           (0x00000010)    /* Latest generated fault memory variable flag; target current fault */
#define FS_BINARY_FLAG_LOP                      (0x00000020)    /* Latest generated fault memory variable flag; loss of phase fault */
#define FS_BINARY_FLAG_TARGET_SPEED             (0x00000040)    /* Latest generated fault memory variable flag; target speed fault */
#define FS_BINARY_FLAG_REF_SPEED                (0x00000080)    /* Latest generated fault memory variable flag; reference speed fault */
#define FS_BINARY_FLAG_EST_SPEED                (0x00000100)    /* Latest generated fault memory variable flag; estimated speed fault */
#define FS_BINARY_FLAG_SP_CONTROLLER            (0x00000200)    /* Latest generated fault memory variable flag; speed conroller fault */
#define FS_BINARY_FLAG_STARTUPOL_DURATION       (0x00000400)    /* Latest generated fault memory variable flag; startup open loop duration fault */
#define FS_BINARY_FLAG_TURNOFFOL_DURATION       (0x00000800)    /* Latest generated fault memory variable flag; turnoff open loop duration fault */
#define FS_BINARY_FLAG_THERMAL_PROTECTION       (0x00001000)    /* Latest generated fault memory variable flag; thermal protection fault */
#define FS_BINARY_FLAG_ONTHEFLY_DURATION        (0x00002000)    /* Latest generated fault memory variable flag; on the fly duration fault*/
#define FS_BINARY_FLAG_CURRENT_INTEGRAL         (0x00004000)    /* Latest generated fault memory variable flag; current integral check fault*/
#define FS_BINARY_FLAG_OVER_LOAD                (0x00008000)    /* Latest generated fault memory variable flag; current integral over load check fault*/
#define FS_BINARY_FLAG_DCBUS_INTEGRAL           (0x00010000)    /* Latest generated fault memory variable flag; high voltage integral check fault */
#define FS_BINARY_FLAG_IPM_TEMP                 (0x00020000)    /* Latest generated fault memory variable flag; high ipm temperature */


#endif
