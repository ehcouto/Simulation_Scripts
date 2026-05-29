#ifndef DSP_H
#define DSP_H


#define MEDIAN_FILTER_BUFFER_SIZE       3       /* read only */
#define MOVING_AVERAGE_BUFFER_SIZE_MAX  128     /* read only */


typedef struct{
    mc_bool_t enabled;
    mc_float_t coeff;
    mc_float_t filtered_data;
} FiltLpf_t;

typedef struct{
    mc_bool_t enabled;
    mc_float_t filtered_data;
    mc_uint32_t indx;
    mc_float_t buf[MEDIAN_FILTER_BUFFER_SIZE];
} FiltMedian_t;

typedef struct{
    mc_bool_t enabled;
    mc_float_t filtered_data;
    mc_uint32_t indx;
    mc_float_t *pbuf;
    mc_uint8_t size;
} FiltMovAve_t;

typedef struct{
    mc_bool_t enabled;
    mc_float_t result;
    mc_float_t sum;
    mc_uint32_t cntr;
    mc_uint32_t cntrMax;
}FiltAve_t;


mc_bool_t FilterLpfInit(FiltLpf_t *_lpf, mc_float_t _coeff);
mc_bool_t FilterLpfHandler(FiltLpf_t *_lpf, mc_float_t _data, mc_float_t *_result);

mc_bool_t FilterMedianInit(FiltMedian_t *medf, mc_float_t startup_data);
mc_bool_t FilterMedianCalc(FiltMedian_t *medf, mc_float_t raw_data, mc_float_t *result);

mc_bool_t FilterMovingAverageInit(FiltMovAve_t *movave, mc_uint8_t size, mc_float_t startup_data);
mc_bool_t FilterMovingAverageCalc(FiltMovAve_t *movave,  mc_float_t raw_data, mc_float_t *result);

void FilterAverageInit(FiltAve_t *ave, mc_uint32_t sampleCount);
mc_float_t FilterAverageCalc(FiltAve_t *ave, mc_float_t new_data);



#endif