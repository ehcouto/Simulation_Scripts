/*
 * Simulation Architecture - Beko FOC Simulation
 * Beko DEA801 Board Parameters
 * */

 //Input Params
 #define VMAINS           230.0f
 #define IPM_TEMPERATURE  25.0f

//Board Params
#define ADC_CURR_GAIN       -1.3636363636f
#define ADC_CURR_OFFSET     1.6343f
#define ADC_VOLT_GAIN       0.007430f
#define LVPS_VOLTAGE        3.3f
#define ADC_BITS            15U
#define ADC_BITS_MAX        (float)((1U << ADC_BITS) - 1U)

// Board Noise Params (Current)
#define ADC_NOISE_MEAN     0.0f
#define ADC_NOISE_STDDEV   0.008f

// Board Noise Params (VBus)
#define ADC_NOISE_MEAN_VBUS     0.0f
#define ADC_NOISE_STDDEV_VBUS   0.005f

//VDC Voltage Drop
#define VDC_VOLTAGE_DROP   5.0f