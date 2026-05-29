#include "mcinclude.h"


void torqCompensationHandlerFL(mcCpv_t* p_mpv)
{
	if((p_mpv->p.torqComp.enabled == MC_TRUE) && (p_mpv->v.state == RAMP))
    { 
        p_mpv->v.torqComp.mPos = p_mpv->v.thact.th;
        p_mpv->v.torqComp.cntrFL = p_mpv->sysHandlerTotalCntFL;
        p_mpv->v.torqComp.cntrFL_delta = p_mpv->v.torqComp.cntrFL - p_mpv->v.torqComp.cntrFL_prev;
        p_mpv->v.torqComp.torq = (mcc_t)0.0f;
        
        if(p_mpv->v.torqComp.mPos < p_mpv->v.torqComp.mPosPrev && p_mpv->v.torqComp.mPos < 20.0f && p_mpv->v.torqComp.mPosPrev > 340.0f)
        {
            p_mpv->v.torqComp.counterInterval = p_mpv->v.torqComp.cntrFL_delta;
            p_mpv->v.torqComp.cntrIntervalBuffer[p_mpv->v.torqComp.intervalIndx] = p_mpv->v.torqComp.counterInterval;
            p_mpv->v.torqComp.cntrIntervalMin = p_mpv->v.torqComp.cntrIntervalBuffer[0];
            
            for(p_mpv->v.torqComp.intervalIndx_temp = 1; p_mpv->v.torqComp.intervalIndx_temp < 3; p_mpv->v.torqComp.intervalIndx_temp++)
            {
                if(p_mpv->v.torqComp.cntrIntervalBuffer[p_mpv->v.torqComp.intervalIndx_temp] < p_mpv->v.torqComp.cntrIntervalMin)
                {
                    p_mpv->v.torqComp.cntrIntervalMin = p_mpv->v.torqComp.cntrIntervalBuffer[p_mpv->v.torqComp.intervalIndx_temp];
                }    
            }
            
            if(p_mpv->v.torqComp.cntrIntervalMin == p_mpv->v.torqComp.counterInterval)
            {
                p_mpv->v.torqComp.mPosOverflowCntr = 0;
            }
            else
            {
                p_mpv->v.torqComp.mPosOverflowCntr++;
            }

            p_mpv->v.torqComp.intervalIndx++;
            if(p_mpv->v.torqComp.intervalIndx >= 3)
            {
                p_mpv->v.torqComp.intervalIndx = 0;
            }

            p_mpv->v.torqComp.cntrFL_prev = p_mpv->v.torqComp.cntrFL;
        }
        p_mpv->v.torqComp.mecPos = p_mpv->v.thact.th + p_mpv->v.torqComp.mPosOverflowCntr * 360.0f;
        
        if(p_mpv->v.torqComp.cntrIntervalBuffer[p_mpv->v.torqComp.intervalIndx] != 0)
        {
            if(p_mpv->p.torqComp.pos_begin > p_mpv->p.torqComp.pos_mid)
            {
                p_mpv->v.torqComp.freqCoeffFirst = 1080 / (p_mpv->p.torqComp.pos_mid - p_mpv->p.torqComp.pos_begin + 1080.0f) / 2.0f;
            }
            else
            {
                p_mpv->v.torqComp.freqCoeffFirst = 1080 / (p_mpv->p.torqComp.pos_mid - p_mpv->p.torqComp.pos_begin) / 2.0f;
            }
            
            if(p_mpv->p.torqComp.pos_mid > p_mpv->p.torqComp.pos_end)
            {
                p_mpv->v.torqComp.freqCoeffSecond = 1080 / (p_mpv->p.torqComp.pos_end - p_mpv->p.torqComp.pos_mid + 1080.0f) / 2.0f;
            }
            else
            {
                p_mpv->v.torqComp.freqCoeffSecond = 1080 / (p_mpv->p.torqComp.pos_end - p_mpv->p.torqComp.pos_mid) / 2.0f;
            }

            if(p_mpv->p.torqComp.pos_mid >= p_mpv->p.torqComp.pos_begin && p_mpv->v.torqComp.mecPos > p_mpv->p.torqComp.pos_begin && p_mpv->v.torqComp.mecPos < p_mpv->p.torqComp.pos_mid)
            {    
                p_mpv->v.torqComp.wt = 2.0f * 3.14f * (p_mpv->v.torqComp.freqCoeffFirst * (p_mpv->v.torqComp.mecPos - p_mpv->p.torqComp.pos_begin) - 270.0f) / 1080.0f;
                p_mpv->v.torqComp.sin = sinf(p_mpv->v.torqComp.wt);
                p_mpv->v.torqComp.torq = (p_mpv->p.torqComp.amp * p_mpv->v.torqComp.sin + p_mpv->p.torqComp.amp) / 2;
            }
            else if(p_mpv->p.torqComp.pos_end >= p_mpv->p.torqComp.pos_mid && p_mpv->v.torqComp.mecPos > p_mpv->p.torqComp.pos_mid && p_mpv->v.torqComp.mecPos < p_mpv->p.torqComp.pos_end)
            {    
                p_mpv->v.torqComp.wt = 2.0f * 3.14f * (p_mpv->v.torqComp.freqCoeffSecond * (p_mpv->v.torqComp.mecPos - p_mpv->p.torqComp.pos_mid) + 270.0f) / 1080.0f;
                p_mpv->v.torqComp.sin = sinf(p_mpv->v.torqComp.wt);
                p_mpv->v.torqComp.torq = (p_mpv->p.torqComp.amp * p_mpv->v.torqComp.sin + p_mpv->p.torqComp.amp) / 2;
            }
            else if(p_mpv->p.torqComp.pos_mid < p_mpv->p.torqComp.pos_begin && p_mpv->p.torqComp.pos_mid < p_mpv->p.torqComp.pos_end && (p_mpv->v.torqComp.mecPos > p_mpv->p.torqComp.pos_begin || p_mpv->v.torqComp.mecPos < p_mpv->p.torqComp.pos_mid))
            {
                if(p_mpv->v.torqComp.mecPos >= p_mpv->p.torqComp.pos_begin)
                {
                    p_mpv->v.torqComp.wt = 2.0f * 3.14f * (p_mpv->v.torqComp.freqCoeffFirst * (p_mpv->v.torqComp.mecPos - p_mpv->p.torqComp.pos_begin) - 270.0f) / 1080.0f;
                }
                else
                {
                    p_mpv->v.torqComp.wt = 2.0f * 3.14f * (p_mpv->v.torqComp.freqCoeffFirst * (1080.0f + p_mpv->v.torqComp.mecPos - p_mpv->p.torqComp.pos_begin) - 270.0f) / 1080.0f;
                }
                p_mpv->v.torqComp.sin = sinf(p_mpv->v.torqComp.wt);
                p_mpv->v.torqComp.torq = (p_mpv->p.torqComp.amp * p_mpv->v.torqComp.sin + p_mpv->p.torqComp.amp) / 2;
            }
            else if(p_mpv->p.torqComp.pos_end < p_mpv->p.torqComp.pos_mid && p_mpv->p.torqComp.pos_begin < p_mpv->p.torqComp.pos_mid && (p_mpv->v.torqComp.mecPos > p_mpv->p.torqComp.pos_mid || p_mpv->v.torqComp.mecPos < p_mpv->p.torqComp.pos_end))
            {
                if(p_mpv->v.torqComp.mecPos >= p_mpv->p.torqComp.pos_mid)
                {
                    p_mpv->v.torqComp.wt = 2.0f * 3.14f * (p_mpv->v.torqComp.freqCoeffSecond * (p_mpv->v.torqComp.mecPos - p_mpv->p.torqComp.pos_mid) + 270.0f) / 1080.0f;
                }
                else
                {
                    p_mpv->v.torqComp.wt = 2.0f * 3.14f * (p_mpv->v.torqComp.freqCoeffSecond * (1080.0f + p_mpv->v.torqComp.mecPos - p_mpv->p.torqComp.pos_mid) + 270.0f) / 1080.0f;
                }
                p_mpv->v.torqComp.sin = sinf(p_mpv->v.torqComp.wt);
                p_mpv->v.torqComp.torq = (p_mpv->p.torqComp.amp * p_mpv->v.torqComp.sin + p_mpv->p.torqComp.amp) / 2;
            }
            else
            {
                p_mpv->v.torqComp.torq = 0.0f;
            }
        }
        else
        {
            p_mpv->v.torqComp.torq = 0.0f;
        }
        p_mpv->v.torqComp.compIq = p_mpv->v.torqComp.compCoeff * p_mpv->v.torqComp.torq;

        p_mpv->v.torqComp.mPosPrev = p_mpv->v.torqComp.mPos;
   }
   else
   {
        p_mpv->v.torqComp.compIq = (mcc_t)0.0f;
   }
}