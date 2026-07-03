/*
 * Simulation Architecture - Beko FOC Simulation
 * Author: EDUARDO HENRIQUE COUTO
 * Requisit: C99, <math.h>
 * Date: May/2026
 *
 * Intended to simulate driver & abstraction layers.           
 */
#include <math.h>
#include <stdlib.h>

#include "brd.h"
#include "pmsm_rk4.h"
#include "Board_Sel.h"
#include "drv.h"

#define MATHCALC__SATURATE_DIRECT(lim_inf, value, lim_sup)                                          \
 (((value) >= (lim_sup)) ? (lim_sup) : ( ((value) <= (lim_inf)) ? (lim_inf) : (value) )) 

#define BRD_CURR_RECONSTRUCTION      TRUE
#define BRD_M_PI		             3.14159265358979323846

static DutyCycleStates DutyCycle;

static uint8_t brd_status = 0;
static float Current_U;
static float Current_V;
static float Current_W;
static uint32_t Curr_U_Raw;
static uint32_t Curr_V_Raw;
static uint32_t Curr_W_Raw;
static uint32_t Vdc_Raw;
static uint32_t Curr_Offset;
static float DcBusVoltage;
static float IpmTemperature;
static bool PWM_Enabled;
static bool IPMFaultStatus;

static PMSMOutputs Motor_Data;

static void Current_Sensor(uint32_t *curr_out, float curr_in);
static void Voltage_Sensor(uint32_t *vdc_raw);
static float noise_gaussian(float mean, float stddev);

/**
* @brief Board layer initialization function
* 
* Function will load all initial values of parameters and variables about board layer. All related 
* sublayers or process related to board layer will be initialized in this function body.* 
*/
void brd_init(void)
{
    //Initalize the Board
    brd_status  = 1;

    Current_U = 0.0f;
    Current_V = 0.0f;
    Current_W = 0.0f;
    DcBusVoltage = 200.0f;
    IpmTemperature = 20.0f;

    //Calculate Current Offset in ADC Counts
    Curr_Offset = (uint32_t)(ADC_CURR_OFFSET * ADC_BITS_MAX / LVPS_VOLTAGE); 

    //Initializing Duties
    DutyCycle.U = 0.0f;
    DutyCycle.V = 0.0f;
    DutyCycle.W = 0.0f;

    PWM_Enabled = false;
    IPMFaultStatus = false;

    ADC_Reading();
}


/* ================= Auxiliary Functions =================== */

/* ================= brd handler to update dat =================== */
void ADC_Reading()
{
    /* ********************************************* */
    /* ************ Load Motor Data **************** */ 
    /* ********************************************* */
    pmsm_get_outputs(&Motor_Data);

    /* ********************************************* */
    /* ************** ADC Sampling ***************** */ 
    /* ********************************************* */
    Current_Sensor(&Curr_U_Raw, Motor_Data.Iu); //Returns Raw Data in ADC Counts
    Current_Sensor(&Curr_V_Raw, Motor_Data.Iv); //Returns Raw Data in ADC Counts
    Current_Sensor(&Curr_W_Raw, Motor_Data.Iw); //Returns Raw Data in ADC Counts
    Voltage_Sensor(&Vdc_Raw); //Returns Raw Data in ADC Counts
}




/* ================= Simulate the ADC Current Sensor =================== */
void Current_Sensor(uint32_t *curr_out, float curr_in)
{
    float temp;
    float noise;

    noise = noise_gaussian(ADC_NOISE_MEAN, ADC_NOISE_STDDEV); // Simulate sensor noise with mean and stddev

    //Calculating Output Current Sensor in [V] 
    temp = (curr_in + noise) * ADC_CURR_GAIN + ADC_CURR_OFFSET;

    //Clamp at LVPS Voltage
    if(temp > LVPS_VOLTAGE)
        temp = LVPS_VOLTAGE;
    else if(temp < 0.0f)
        temp = 0.0f;
    
    //Return Data in ADC Counts
    *curr_out = (uint32_t)(temp*ADC_BITS_MAX/LVPS_VOLTAGE);
}




/* ================= Simulate the ADC Voltage Sensor =================== */
void Voltage_Sensor(uint32_t *vdc_raw)
{
    float temp;
    float vpeak;
    float noise;

    //Add gaussian noise
    noise = noise_gaussian(ADC_NOISE_MEAN_VBUS , ADC_NOISE_STDDEV_VBUS); // Simulate sensor noise with mean and stddev

    //Peak Voltage Calculation + Noise
    vpeak = (VMAINS * sqrtf(2.0f) - VDC_VOLTAGE_DROP) + noise; //VDC Calc + Noise
    
    //Calculating Output Voltage Sensor in [V]
    temp = (float)(vpeak * ADC_VOLT_GAIN);

    //Clamp at LVPS Voltage
    if(temp > LVPS_VOLTAGE)
        temp = LVPS_VOLTAGE;
    else if(temp < 0.0f)
        temp = 0.0f;

    //Return Data in ADC Counts
    temp = temp / LVPS_VOLTAGE;
    *vdc_raw = (uint32_t)(temp * ADC_BITS_MAX);
}




/* ================= Gaussian Noise Generator =================== */
float noise_gaussian(float mean, float stddev)
{
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);

    float mag = stddev * sqrtf(-2.0f * logf(u1));
    float z0  = mag * cosf(2.0f * BRD_M_PI * u2) + mean;

    return z0;
}




/* ================= API Function =================== */
void brdGetData(float *current_u, float *current_v, float *current_w, float *dc_bus_voltage, float *ipm_temperature)
{
    *current_u = Current_U;
    *current_v = Current_V;
    *current_w = Current_W;
    *dc_bus_voltage = DcBusVoltage;
    *ipm_temperature = IpmTemperature;
}




DutyCycleStates brdGetDuties(void)
{
    DutyCycleStates hw_duty;

    if(PWM_Enabled == true)
    {
        hw_duty.U = DutyCycle.U;
        hw_duty.V = DutyCycle.V;
        hw_duty.W = DutyCycle.W;
    }
    else
    {
        hw_duty.U = 0.0f;
        hw_duty.V = 0.0f;
        hw_duty.W = 0.0f; 
    }

    return(hw_duty);
}




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
    float U_lim, V_lim, W_lim;

        //Apply Duty Cycle Limitation
        U_lim = MATHCALC__SATURATE_DIRECT((1.0f - DUTY_CYCLE_LIMIT), dutyU, DUTY_CYCLE_LIMIT);
        V_lim = MATHCALC__SATURATE_DIRECT((1.0f - DUTY_CYCLE_LIMIT), dutyV, DUTY_CYCLE_LIMIT);
        W_lim = MATHCALC__SATURATE_DIRECT((1.0f - DUTY_CYCLE_LIMIT), dutyW, DUTY_CYCLE_LIMIT);

    // Set Suty cycles
    DutyCycle.U = U_lim;
    DutyCycle.V = V_lim;
    DutyCycle.W = W_lim;
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
    if((sec == 255) &&
       (ADC_CURR_GAIN < 0.0f)) //Negative Gain
    {
        //If positive nothing changes, but if negative only invert signals (reconstruction disabled)
            if(ADC_CURR_GAIN < 0.0f) //Negative Gain
            {
                Current_U = -Current_U;
                Current_W = -Current_W;
                Current_V = -Current_V;
            }
    }
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
/* ********************************************* */
    /* ************** SW Routine ***************** */ 
    /* ********************************************* */
    IpmTemperature = IPM_TEMPERATURE; //No simulation for IPM Temperature... Keep it at pre-defined & fixed value (25C)
    DcBusVoltage = Vdc_Raw/VOLTAGE_AMP*ADC_STEP; //to do: Create a model for the Vdc voltage
    
    Current_U = ((float)Curr_U_Raw - (float)Curr_Offset)/(float)CURRENT_AMP*(float)ADC_STEP;
    Current_V = ((float)Curr_V_Raw - (float)Curr_Offset)/(float)CURRENT_AMP*(float)ADC_STEP;
    Current_W = ((float)Curr_W_Raw - (float)Curr_Offset)/(float)CURRENT_AMP*(float)ADC_STEP;

    if(BRD_CURR_RECONSTRUCTION == FALSE)
    {
        sec = 255;
    }

    //Current Reconstruction
    switch(sec)
    {
      case 2 :
      case 3 :
        {
                if(ADC_CURR_GAIN > 0.0f) //Positive Gain
                {
                    Current_W = -(Current_U + Current_V);
                } 
                else if(ADC_CURR_GAIN < 0.0f) //Negative Gain
                {
                    Current_U = -Current_U;
                    Current_V = -Current_V;
                    Current_W = -(Current_U + Current_V);
                }
        }
        break;
      case 4 :
      case 5 :
        {
                if(ADC_CURR_GAIN > 0.0f) //Positive Gain
                {
                    Current_U = -(Current_W + Current_V);
                } 
                else if(ADC_CURR_GAIN < 0.0f) //Negative Gain
                {
                    Current_V = -Current_V;
                    Current_W = -Current_W;
                    Current_U = -(Current_W + Current_V);
                }
        }
        break;
      case 6 :
      case 1 :
        {
                if(ADC_CURR_GAIN > 0.0f) //Positive Gain
                {
                    Current_V = -(Current_W + Current_U);

                } 
                else if(ADC_CURR_GAIN < 0.0f) //Negative Gain
                {
                    Current_U = -Current_U;
                    Current_W = -Current_W;
                    Current_V = -(Current_W + Current_U);
                }
        }
        break;
      default :
        {
            brdMapAdcChannels(sec);
        }
    }
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
    return(IPMFaultStatus);
}


void brdResetIpmFaultOutState(void)
{
    IPMFaultStatus = false;
}


void brdPwmEnable(void)
{
    PWM_Enabled = true;
}


void brdPwmDisable(void)
{
    PWM_Enabled = false;
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
    *_pos = 0.0001f * *_sp; //Do nothing.. Just to remove warning - Not used.
    return(false);
} 


void brdShortCircuitBottomTransistors(void)
{

}




void SetIPMFault(void)
{
    IPMFaultStatus = true;
}



bool IsPWMEnabled(void)
{
    return(PWM_Enabled);
}