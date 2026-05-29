#include "utility.h"


#define _MOVING_AVERAGE_BUFFER_SIZE_MAX_        128


mc_bool_t FilterLpfInit(FiltLpf_t *_lpf, mc_float_t _coeff)
{
    if(_coeff > 1)
    {
        return MC_FALSE;
    }
    else
    {
        _lpf->coeff = _coeff;
        _lpf->filtered_data = 0;
        _lpf->enabled = MC_TRUE;
        
        return MC_TRUE;
    }
}

mc_bool_t FilterLpfHandler(FiltLpf_t *_lpf, mc_float_t _data, mc_float_t *_result)
{
    if(_lpf->enabled == MC_FALSE)
    {
        return MC_FALSE;
    }
    else
    {
        _lpf->filtered_data += _lpf->coeff * (_data - _lpf->filtered_data);
        
        *_result = _lpf->filtered_data;
        
        return MC_TRUE;
    }
}

mc_bool_t FilterMedianInit(FiltMedian_t *medf, mc_float_t startup_data)
{
    int indx = 0;
    
    if(MEDIAN_FILTER_BUFFER_SIZE != 3)
    {
        return MC_FALSE;
    }
    else
    {
        for(indx = 0; indx < MEDIAN_FILTER_BUFFER_SIZE; indx++)
        {
            medf->buf[indx] = startup_data;
        }
        
        medf->indx = 0;
        medf->filtered_data = startup_data;
        
        medf->enabled = MC_TRUE;
        
        return MC_TRUE;
    }
}

mc_bool_t FilterMedianCalc(FiltMedian_t *medf, mc_float_t raw_data, mc_float_t *result)
{
    int indx = 0;
    
    if(medf->enabled == MC_FALSE)
    {
        return MC_FALSE;
    }
    else
    {
        medf->buf[medf->indx] = raw_data;
        
        if ((medf->buf[indx] > medf->buf[indx + 1] &&
             medf->buf[indx] < medf->buf[indx + 2]) ||
            (medf->buf[indx] < medf->buf[indx + 1] &&
             medf->buf[indx] > medf->buf[indx + 2]))
        {
            medf->filtered_data = medf->buf[indx];
        }
        else if ((medf->buf[indx + 1] > medf->buf[indx + 2] &&
                  medf->buf[indx + 1] < medf->buf[indx]) ||
                 (medf->buf[indx + 1] < medf->buf[indx + 2] &&
                  medf->buf[indx + 1] > medf->buf[indx]))
        {
            medf->filtered_data = medf->buf[indx + 1];
        }
        else if ((medf->buf[indx + 2] > medf->buf[indx] &&
                  medf->buf[indx + 2] < medf->buf[indx + 1]) ||
                 (medf->buf[indx + 2] < medf->buf[indx] &&
                  medf->buf[indx + 2] > medf->buf[indx + 1]))
        {
            medf->filtered_data = medf->buf[indx + 2];
        }
        else
        {
            ;   /* do nothing */
        }
        
        if (++medf->indx >= MEDIAN_FILTER_BUFFER_SIZE)
        {
            medf->indx = 0;
        }
        else
        {
            ;   /* do nothing */
        }
        
        *result = medf->filtered_data;
        
        return MC_TRUE;
    }
}

mc_bool_t FilterMovingAverageInit(FiltMovAve_t *movave, mc_uint8_t size, mc_float_t startup_data)
{
    int indx = 0;
    
    if((size < 2) || (MOVING_AVERAGE_BUFFER_SIZE_MAX != _MOVING_AVERAGE_BUFFER_SIZE_MAX_) || 
       (size > MOVING_AVERAGE_BUFFER_SIZE_MAX))
    {
        return MC_FALSE;
    }
    else
    {
        movave->size = size;
        
        for(indx = 0; indx < movave->size; indx++)
        {
            movave->pbuf[indx] = startup_data;
        }
        
        movave->filtered_data = startup_data;
        movave->indx = 0;
        
        movave->enabled = MC_TRUE;
        
        return MC_TRUE;
    }
}

mc_bool_t FilterMovingAverageCalc(FiltMovAve_t *movave,  mc_float_t raw_data, mc_float_t *result)
{
    mc_int32_t indx = 0;
    mc_float_t sum = 0;
    
    if(movave->enabled == MC_FALSE)
    {
        return MC_FALSE;
    }
    else
    {
        movave->pbuf[movave->indx] = raw_data;
        
        for(indx = 0, sum = 0; indx < movave->size; indx++)
        {
            sum += movave->pbuf[indx];
        }
        movave->filtered_data = sum / (mc_float_t)indx;
        
        if(++movave->indx >= movave->size)
        {
            movave->indx = 0;
        }
        else
        {
            ;   /* do nothing */
        }
        
        *result = movave->filtered_data;
        
        return MC_TRUE;
    }
}

void FilterAverageInit(FiltAve_t *ave, mc_uint32_t sampleCount)
{
    ave->sum = 0;
    ave->cntr = 0;
    ave->cntrMax = sampleCount;
    ave->enabled = 1;
}

mc_float_t FilterAverageCalc(FiltAve_t *ave, mc_float_t new_data)
{
    ave->sum += new_data;
    ave->cntr++;
    if(ave->cntr >= ave->cntrMax)
    {
	ave->result = ave->sum / ave->cntr;
	ave->sum = 0;
	ave->cntr = 0;
    }    
    return ave->result;
}
