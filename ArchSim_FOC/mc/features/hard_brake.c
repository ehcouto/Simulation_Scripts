#include "mcinclude.h"

void mcHwHardBrakeHandlerSL(mcCpv_t *p_mpv)
{
    if((p_mpv->p.hwHardBrake.enabled == MC_TRUE) && (p_mpv->v.acc == ACC_NEG))
    {
       if((p_mpv->v.spref_abs > p_mpv->p.hwHardBrake.minsp) && 
          (p_mpv->v.spref_abs < p_mpv->p.hwHardBrake.maxsp) &&
              (p_mpv->v.sprefin_abs < p_mpv->p.hwHardBrake.minsp))
       {
           p_mpv->v.hwHardBrake.state = MC_HW_HB_STATE_ACTIVE;
           
           p_mpv->v.idqref.d = p_mpv->v.idqref.q = 0.0f;
           p_mpv->v.vab.a = p_mpv->v.vab.b = 0.0f;
           p_mpv->v.vdq.d = p_mpv->v.vdq.q = 0.0f;
           p_mpv->v.vdq_decoupler_dq.d = p_mpv->v.vdq_decoupler_dq.q = 0.0f;
               
           p_mpv->v.spref = p_mpv->v.spest;
           
           p_mpv->v.piSpeed.integ = 0.0f;
           p_mpv->v.piDCurrent.integ = 0.0f;
           p_mpv->v.piQCurrent.integ = 0.0f;
       }
       else if(p_mpv->v.hwHardBrake.state == MC_HW_HB_STATE_ACTIVE)
       {
           /* set integral components */
           p_mpv->v.hwHardBrake.state = MC_HW_HB_STATE_PASSIVE;
       }
       else
       {
           p_mpv->v.hwHardBrake.state = MC_HW_HB_STATE_PASSIVE;
       }
    }
    else
    {
        p_mpv->v.hwHardBrake.state = MC_HW_HB_STATE_PASSIVE;
    }
}
