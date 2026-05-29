#include "mcinclude.h"

#ifdef ENABLE_MCTEST_OVERWRITE_VAB

/*
	Check PWMs and phase currents via oscilloscope.
	Use after inverse park transform to overwrite calculated vab.
	Code example (foc.c):

	InvParkTransform((mcdq_t*)&p_mpv->v.vdq, &p_mpv->v.thact, (mcab_t*)&p_mpv->v.vab);
	mcTestOverwriteVabFL(p_mpv, &p_mpv->v.vab);
*/

mc_float_t mctest_f = 1.0f, mctest_vamp = 0.0f; 
mc_float_t mctest_t, mctest_T, mctest_wt, mctest_va, mctest_vb;

void mcTestOverwriteVabFL(mindx_t l_indx, mcCpv_t *p_mpv, mcab_t* l_vab)
{
	if(p_mpv->mindx != l_indx)
		return;
	
	p_mpv->v.startup.ol.tValidSpCnt = 0;

	mctest_t += p_mpv->v.periodFL;
	
	mctest_T = 1 / mctest_f;
	
	if(mctest_t > mctest_T)
		mctest_t = mctest_t - mctest_T;
	    
	mctest_wt = MC_TWOPI * mctest_f * mctest_t;
    
	mctest_va = mctest_vamp * cosf(mctest_wt);
	mctest_vb = mctest_vamp * sinf(mctest_wt);
    
	l_vab->a = mctest_va;
	l_vab->b = mctest_vb;
}

#elif defined(ENABLE_MCTEST_CHECK_CURRENT_CONTROLLER)

/*
	Check if current controller activates as desired. 
	Verification should be applied via oscilloscope.

	Code example (slow loop hanler in mc_state.c):
	mcTestCheckCurrentControllerInOL(p_mpv);
*/

mcdq_t mctest_idq = {0, 0};

void mcTestCheckCurrentControllerInOL(mindx_t l_indx, mcCpv_t *p_mpv)
{
	if(p_mpv->mindx != l_indx)
		return;
	
	p_mpv->v.startup.ol.tValidSpCnt = 0;

	//p_mpv->p.startup.ol.idqmax = mctest_idq;
}

#endif