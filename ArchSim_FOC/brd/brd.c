
#include "brd.h"

static uint8_t brd_status = 0;
static float Current_U;
static float Current_V;
static float Current_W;
static float DcBusVoltage;
static float IpmTemperature;
/**
* @brief Board layer initialization function
* 
* Function will load all initial values of parameters and variables about board layer. All related 
* sublayers or process related to board layer will be initialized in this function body.* 
*/
void brd_init(void)
{
    brd_status  = 1;

    Current_U = 0.0f;
    Current_V = 0.0f;
    Current_W = 0.0f;
    DcBusVoltage = 0.0f;
    IpmTemperature = 0.0f;
}


/**
*
*
*/



/**
* @brief Enable Phase U related PWM output
* 
* Function enables PWM module related output for Phase U
*/
void brdPwmEnablePhaseU(void)
{

}


/**
* @brief Enable Phase V related PWM output
* 
* Function enables PWM module related output for Phase V
*/
void brdPwmEnablePhaseV(void)
{

}


/**
* @brief Enable Phase W related PWM output
* 
* Function enables PWM module related output for Phase W
*/
void brdPwmEnablePhaseW(void)
{

}


/**
* @brief Enable Phases UVW related PWM output
* 
* Function enables PWM module related output for Phase W
*/
void brdPwmEnablePhaseUVW(void)
{

}


/**
* @brief Set PWM duty cycles
*
* Function sets PWM duties as the given function parameters. Float type parameters are desired
*
* @param[in]    dutyU   duty cycle value for phase U
* @param[in]    dutyV   duty cycle value for phase V
* @param[in]    dutyW   duty cycle value for phase W
*/
void brdSetPwmDuties(float dutyU, float dutyV, float dutyW)
{

}


/**
* @brief Turn on inrush relay
*
* Supplies bara voltage via inrush relay
*
* @return       boolean value shows success status of the function; any case returns true
*/
void brdTurnOnInrushRelay(void)
{

}

/**
* @brief Turn off inrush relay
*
* Disconnects bara voltage to main supply
*
* @return       boolean value shows success status of the function; any case returns true
*/
void brdTurnOffInrushRelay(void)
{

}

/**
* @brief map adc channels
* 
* Function is an interface for driver side to map correct adc channels for the first parallel sampling 
*
* @return       float type phase U current
*/
void brdMapAdcChannels(uint8_t sec)
{

}


/**
* @brief sample adc channels
* 
* Function samples adc channels according to current sector 
*
* @return       no return value
*/
void brdSampleCurrentsUVW(uint8_t sec)
{

}


/**
* @brief samples phase U current
* 
* Function enables PWM module related output for Phase U. Dividing resistors; defined
* as board parameters; are important for the resultant sampled current. 
*
* @return       float type phase U current
*/
float brdGetPhaseCurrentU(void)
{
    return(Current_U);
}


/**
* @brief samples phase V current
* 
* Function enables PWM module related output for Phase V. Dividing resistors; defined
* as board parameters; are important for the resultant sampled current. 
*
* @return       float type phase V current
*/
float brdGetPhaseCurrentV(void)
{
    return(Current_V);
}


/**
* @brief samples phase W current; from ADC module "a" 
* 
* Function enables PWM module related output for Phase W. Dividing resistors; defined
* as board parameters; are important for the resultant sampled current. 
*
* @return       float type phase W current
*/
float brdGetPhaseCurrentW(void)
{
    return(Current_W);
}


/**
* @brief samples dc bus voltage level 
* 
* Function samples related ADC for dc bus voltage level and calculate the
* voltage as "volt" unit
* @return       float type phase W current
*/
float brdGetDcBusVoltage(void)
{
    return(DcBusVoltage);
}


bool brdCurrentCalibrationInit(void)
{
    return(true);
}

bool brdCurrentCalibrationHandler(void)
{
    return(true);
}


float brdGetIpmTemperature(void)
{
    return(IpmTemperature); 
}


bool brdGetIpmFaultOutState(void)
{
    return(false);
}


void brdResetIpmFaultOutState(void)
{

}


void brdPwmEnable(void)
{

}


void brdPwmDisable(void)
{

}


void brdWatchdogResetHandlerFL(void)
{

}

void brdResetWdg(void)
{

}

void brdEnableInterrupts(void)
{

}

void brdDisableInterrupts(void)
{

}


bool brdGetDCBusOpenStatus(void)
{
    return(false);
}

bool brdGetEncoderData(float *_pos, float *_sp)
{
    return(false);
}


void brdShortCircuitBottomTransistors(void)
{

}





/* ================= Auxiliary Functions =================== */

void brdSetData(float current_u, float current_v, float current_w, float dc_bus_voltage, float ipm_temperature)
{
    Current_U = current_u;
    Current_V = current_v;
    Current_W = current_w;
    DcBusVoltage = dc_bus_voltage;
    IpmTemperature = ipm_temperature;
}