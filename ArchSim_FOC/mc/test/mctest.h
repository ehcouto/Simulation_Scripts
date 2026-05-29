#ifndef MCTEST_H
#define MCTEST_H

//#define ENABLE_MCTEST_OVERWRITE_VAB
//#define ENABLE_MCTEST_CHECK_CURRENT_CONTROLLER

#ifdef ENABLE_MCTEST_OVERWRITE_VAB
void mcTestOverwriteVabFL(mindx_t l_indx, mcCpv_t *p_mpv, mcab_t* l_vab);
#elif defined(ENABLE_MCTEST_CHECK_CURRENT_CONTROLLER)
void mcTestCheckCurrentControllerInOL(mindx_t l_indx, mcCpv_t *p_mpv);
#endif

#endif
