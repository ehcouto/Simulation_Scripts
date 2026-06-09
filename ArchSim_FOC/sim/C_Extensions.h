#ifndef C_EXTENSIONS_H
    #define C_EXTENSIONS_H

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================
//---------------------------------------------------------------------------------------------------------------------
//--------------------------------------------- STANDARD SYSTEM DEFINITIONS -------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

typedef enum _BOOL_TYPE
{
    FALSE           = 0,
    TRUE            = 1
} BOOL_TYPE;

typedef enum _ON_OFF_TYPE
{
    OFF             = 0,
    ON              = 1
} ON_OFF_TYPE;

typedef enum _PASS_FAIL_TYPE
{
    FAIL            = 0,
    PASS            = 1
} PASS_FAIL_TYPE;

#define OK              ((PASS_FAIL_TYPE)1)

typedef enum _COMPLETE_TYPE
{
    INCOMPLETE      = 0,
    COMPLETE        = 1
} COMPLETE_TYPE;

typedef enum _ACTIVE_TYPE
{
    INACTIVE        = 0,
    ACTIVE          = 1
} ACTIVE_TYPE;


//A constant pointer to a function with no return value or parameters.
typedef void (* const FUNCTION_TYPE)(void);

//A constant pointer to a list of TASK_TYPESs.
typedef void (* const * const FUNCTION_LIST_TYPE)(void);

//A NULL pointer to use as a terminator for function lists.
#define NULL_TASK ((FUNCTION_TYPE)0)


#ifndef ENABLED
    #define ENABLED         1
#endif //ENABLED

#ifndef DISABLED
    #define DISABLED        0
#endif //DISABLED

#ifndef NULL
    #define NULL            (void *)0
#endif //NULL

#ifndef UNAVAILABLE
    #define UNAVAILABLE     10
#endif //UNAVAILABLE

#ifndef AVAILABLE
    #define AVAILABLE       11
#endif //AVAILABLE

//---------------------------------------------------------------------------------------------------------------------
//------------------- --------------------------BYTE AND WORD ACCESS --------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

#define LOWORD(mem)         (unsigned short)((mem) & 0xFFFFU)   ///< This macro is for extracting lower word from a double word type of data
#define LOBYTE(mem)         (unsigned char)((mem) & 0xFFU)      ///< This macro is for extracting lower byte from the word type of data
#define HIWORD(mem)         (unsigned short)(((unsigned long)(mem) >> 16U ) & 0xFFFFU)  ///< This macro is for extracting higher word from a double word type of data
#define HIBYTE(mem)         (unsigned char)(((unsigned short)(mem) >> 8U) & 0xFFU)      ///<This macro is for extracting higher byte from a word type of data

#define ELEMENTS_IN_ARRAY(a)        (sizeof(a) / sizeof((a)[0]))
#define COMBINE_NIBBLES(hi,lo)      ((((hi) << 4) & 0xF0) + ((lo) & 0x0F))
#define COMBINE_BYTES(hi,lo)        ((((hi) << 8) & 0xFF00) + ((lo) & 0xFF))
#define COMBINE_WORDS(hi,lo)        ((((hi) << 16) & 0xFFFF0000) + ((lo) & 0xFFFF))

//---------------------------------------------------------------------------------------------------------------------
//--------------------------------------------- BIT MANIPULATION ------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

#define BIT_TEST(mem,bit)               (((mem)>>(bit))&(1UL))    ///< This macro is used to perform test on bit of the data
#define BIT_SET(mem,bit)                ((mem)|=(1UL<<(bit)))   ///< This macro is used to set the bit of the data
#define BIT_CLR(mem,bit)                ((mem)&=~(1UL<<(bit)))  ///< This macro is used to clear the bit of the data
#define BIT_TOGGLE(mem,bit)             ((mem)^=(1UL<<(bit)))   ///< This macro is used to toggle the bit of the data
#define BIT_STORAGE_SIZE(num_of_bits)   ((size_t)(((num_of_bits) + 7U) / 8U)) ///< this macro returns the number of bytes needed to store the amount bits requested.

//---------------------------------------------------------------------------------------------------------------------
//--------------------------------------------- DATA ALIAS ------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
typedef unsigned char       uint8;
typedef unsigned short int  uint16;
typedef unsigned long int   uint32;
typedef unsigned long long  uint64;

typedef signed char         sint8;
typedef signed short int    sint16;
typedef signed long int     sint32;
typedef signed long long    sint64;

typedef float               float32;
typedef double              float64;

typedef enum
{
    DATATYPE_INVALID = 0,   //!< DATATYPE_INVALID
    DATATYPE_UINT8,         //!< DATATYPE_UINT8
    DATATYPE_UINT16,        //!< DATATYPE_UINT16
    DATATYPE_UINT32,        //!< DATATYPE_UINT32
    DATATYPE_UINT64,        //!< DATATYPE_UINT64
    DATATYPE_SINT8,         //!< DATATYPE_SINT8
    DATATYPE_SINT16,        //!< DATATYPE_SINT16
    DATATYPE_SINT32,        //!< DATATYPE_SINT32
    DATATYPE_SINT64,        //!< DATATYPE_SINT64
    DATATYPE_FLOAT32,       //!< DATATYPE_FLOAT32
    DATATYPE_FLOAT64,       //!< DATATYPE_FLOAT64
    DATATYPE_IS_ARRAY,      //!< DATATYPE_IS_ARRAY Use this type if you want to extern an array. Receiver will need to understand what this array is.
    DATATYPE_IS_REGULATION,  //!< DATATYPE_IS_REGULATION Use this type to extern the Regulation array.
	DATATYPE_IS_Q15_CELSIUS //!< DATATYPE_IS_Q15_CELSIUS is the special type of SINT16 to represent temperatures in celsius on the Q15 format.
}DATATYPE_TYPE;


//---------------------------------------------------------------------------------------------------------------------
//------------------------------------------ DEFINE LOAD TYPES --------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#define LOAD_TYPE_EMPTY               0
#define LOAD_TYPE_SPD_LINEAR          1
#define LOAD_TYPE_SPD_QUADRATIC       2
#define LOAD_TYPE_HYDRAULIC_SIMPLE    3
#define LOAD_TYPE_HYDRAULIC_COMPLETE  4


#endif //C_EXTENSIONS_H
