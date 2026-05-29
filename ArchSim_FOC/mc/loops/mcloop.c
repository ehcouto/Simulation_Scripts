#include "mcinclude.h"


void mcFastLoopHandlerMx(mcCpv_t *p_mpv)
{
    p_mpv->v.loopActiveFL = MC_TRUE;
    
    p_mpv->v.fl2slCnt++;
    if(p_mpv->v.loopActiveSL == MC_TRUE)
    {
        p_mpv->v.flInSlCnt++;
        if(p_mpv->v.flInSlCnt > p_mpv->v.flInSlCntMax)
        {
            p_mpv->v.flInSlCntMax = p_mpv->v.flInSlCnt;
        }
        else
        {
            /* do nothing */
        }
    }
    else
    {
        /* do nothing */
    }
    
    mcSetStateCommand(p_mpv);
    mcStateMachineHandlerFL(p_mpv);
    mcFocFastLoop(p_mpv);
    p_mpv->func->mcBrdMapAdcChannels(p_mpv->v.sector);
    
    if(p_mpv->v.pwm_enabled == MC_TRUE)
    {
        if(p_mpv->v.hwHardBrake.state == MC_HW_HB_STATE_ACTIVE)
        {
            p_mpv->func->mcBrdShortCircBotTransistors();
        }
        else
        {
            p_mpv->func->mcBrdSetPwmDuties((mc_float_t)p_mpv->v.duvw_comp.u, 
                                          (mc_float_t)p_mpv->v.duvw_comp.v,
                                          (mc_float_t)p_mpv->v.duvw_comp.w);
        }
    }
    else
    {
        /* do nothing */
    }
    
    p_mpv->sysHandlerTotalCntFL++;

    p_mpv->v.loopActiveFL = MC_FALSE;
}

void mcSlowLoopHandlerMx(mcCpv_t *p_mpv)
{
    p_mpv->v.loopActiveSL = MC_TRUE;

    p_mpv->v.fl2slCnt = 0u;

    mcStateMachineHandlerSL(p_mpv);
    if(p_mpv->v.pwm_enabled == MC_TRUE)
    	mcFocSlowLoop(p_mpv);

    p_mpv->sysHandlerTotalCntSL++;

    p_mpv->v.loopActiveSL = MC_FALSE;
    p_mpv->v.flInSlCnt = 0u;
}

void mcMainLoopHandlerMx(mcCpv_t *p_mpv)
{
    p_mpv->sysHandlerTotalCntML++;
}

