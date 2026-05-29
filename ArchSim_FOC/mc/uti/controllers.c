#include "utility.h"

/**
* @brief Speed PI controller
*
* Calculates target iqref for the decided reference speed. Gap between decided reference speed and current speed is the
* input of the controller. Ambition to make difference zero.
*
* @param[in]    _error                  error or gap between reference value and actual estimated or sensed value
* @param[in]   _piv                     address for calculated reference speed variable
*
@return         calculated pi controller  result
*/
mc_float_t contDiscretePiController(mc_float_t _error, contPiController_t* _piv)
{
    mc_float_t _excess;
    mc_float_t _tempInteg;
    
    _tempInteg = _piv->integ;
    _tempInteg += _piv->ki * _error;
    
    _excess = (mc_float_t)0;
    if(_tempInteg > (mc_float_t)0 && _tempInteg > _piv->antiWindupLimit) 
    {
        _excess = _tempInteg - _piv->antiWindupLimit;
    }
    else if(_tempInteg < (mc_float_t)0 && _tempInteg < -_piv->antiWindupLimit)
    {
        _excess = _tempInteg + _piv->antiWindupLimit;
    }
    else
    {
        ;       /* do nothing */
    }
    _tempInteg = _tempInteg - _piv->antiWindupCoef * _excess;
    
    _piv->error = _error;
    _piv->prop = _piv->kp * _error;
    _piv->integ = _tempInteg;

    return _piv->prop + _piv->integ;
}

/**
* @brief Saturation check and correction
*
* check if the current value is in the boundary between defined minimum value and maximum value. If current value 
* is more than predefined maximum value, sets variable to permitted maximum value and if current value is less 
* than predefined minimum value, sets variable to permitted minimum value.
*
* @param[in]    value   error or gap between reference value and actual estimated or sensed value
* @param[in]    vmin    predefined (permitted) minimum value
* @param[in]    vmax    predefined (permitted) maximum value
*
@return         saturated result
*/
mc_float_t contSaturate(mc_float_t value, mc_float_t vmin, mc_float_t vmax)
{
    if (value > vmax)
    {
        return vmax;
    }
    else if (value < vmin)
    {
        return vmin;
    }
    else
    {
        return value;
    }
}

mc_float_t contCheckAngle0to360deg(mc_float_t value)
{
    if (value >= 360.0f)
    {
        return (value - 360.0f);
    }
    else if (value < 0.0f)
    {
        return (value + 360.0f);
    }
    else
    {
        return value;
    }
}
