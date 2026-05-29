/**
*
*
*/

#ifndef FS_H
#define FS_H


/******************************************************************************/
/* function declerations */

void fsInitFuncSafety(void);
void fsAddMxFuncSafety(mcCpv_t *p_mpv, fsSpv_t *p_spv, const fsPar_t *p_fsp, mcMotorFunc_t *func);

void fsFastLoopHandlerMx(fsSpv_t *p_spv);
void fsSlowLoopHandlerMx(fsSpv_t *p_spv);
void fsHandlerML(fsSpv_t *p_spv);

/* function declerations */
/******************************************************************************/

#endif
