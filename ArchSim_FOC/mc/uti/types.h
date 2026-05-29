/**
*
*
*/

#ifndef TYPES_H
#define TYPES_H
     

typedef unsigned char mc_bool_t;

typedef signed char     mc_int8_t;
typedef unsigned char   mc_uint8_t;

typedef signed short    mc_int16_t;
typedef unsigned short  mc_uint16_t;

typedef signed int      mc_int32_t;
typedef unsigned int    mc_uint32_t;

typedef float mc_float_t;
typedef double mc_double_t;

typedef char* mc_string_t;

typedef enum{
    MC_SUCCESS,
    MC_FAILURE
} achv_t;    ///< type for achivement states


#define SIZE_OF_BOOL    1
#define SIZE_OF_8BIT    1
#define SIZE_OF_16BIT   2
#define SIZE_OF_32BIT   4
#define SIZE_OF_FLOAT   4


#define MC_FALSE       ((mc_bool_t)0x0U)
#define MC_TRUE        ((mc_bool_t)0x1U)

#define MC_DISABLED    ((mc_bool_t)0x0U)
#define MC_ENABLED     ((mc_bool_t)0x1U)

#define MC_NULL        ((void *)0)

#endif
