#ifndef MCCONFIG_H
#define MCCONFIG_H

#define MOTOR_NUMBER_TYPE_FLOATING

//#define ENABLE_WM_ORCA
//#define ENABLE_DRY_ORCA
//#define ENABLE_WM_ORCA_FREE
//#define ENABLE_RAC_FAN
//#define ENABLE_RAC_COMPRESSOR
//#define ENABLE_REF_COMPRESSOR
#define ENABLE_DW_CIRC
#define ENABLE_DW_DRAIN

#ifdef ENABLE_REF_COMPRESSOR
#define SUPPORTED_TOTAL_MOTOR           (1u)
#else
#define SUPPORTED_TOTAL_MOTOR           (2u)
#endif

#endif
