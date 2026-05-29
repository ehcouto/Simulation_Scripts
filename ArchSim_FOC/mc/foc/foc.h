/**
*
*
*/

#ifndef FOC_H
#define FOC_H

#include "mcinclude.h"

/* flag masks to decide state flow */
/* #define FOC_SAMPLING_PROCESS            0X0001 */
#define FOC_CLARK_TRANSFORM             ((mcstc_t)0X0002)
#define FOC_PARK_TRANSFORM              ((mcstc_t)0X0004)    
#define FOC_CURRENT_CONTROLLER          ((mcstc_t)0X0008)
#define FOC_INVERSE_PARK_TRANSFORM      ((mcstc_t)0X0010)
#define FOC_DCBUS_RIPPLE_ELEMINATION    ((mcstc_t)0x0020)
#define FOC_SPACE_VECTOR_MODULATION     ((mcstc_t)0X0040)
#define FOC_DEAD_TIME_COMPENSATION      ((mcstc_t)0X0080)
#define FOC_SPEED_CONTROLLER            ((mcstc_t)0X0100)
#define FOC_RAMP_CONTROLLER             ((mcstc_t)0X0200)
#define FOC_RAMP_SPEED_ESTIMATION       ((mcstc_t)0X0400)
#define FOC_FIELD_WEAKENING             ((mcstc_t)0X0800)
#define FOC_MTPA                        ((mcstc_t)0X1000)

/* #define INDX_PHASE_U    0 */
/* #define INDX_PHASE_V    1 */
/* #define INDX_PHASE_W    2 */


void mcFocFastLoop(mcCpv_t* p_mpv);
void mcFocSlowLoop(mcCpv_t* p_mpv);

void ClarkTransform(const mcuvw_t* l_iuvw, mcab_t* l_iab);
void InvClarkTransform(const mcab_t* l_iab, mcuvw_t* l_iuvw);
void ParkTransform(const mcab_t* l_ab, const th_t* l_thact, mcdq_t* l_dq);
void InvParkTransform(const mcdq_t* l_dq, const th_t* l_thact, mcab_t* l_ab);
mc_bool_t DCBusRippleElimination(const mcab_t* l_vab, mcab_t* l_vabComp, mcv_t l_dcBaraVoltage);
void SpaceVectorModulation(const mcab_t* l_vabComp, sector_t *l_sector, duvw_t* l_pwm_duty);
void SpaceVectorModulationU0(const mcab_t* l_vabComp, sector_t *l_sector, duvw_t* l_pwm_duty);
void SpaceVectorModulationU7(const mcab_t* l_vabComp, sector_t *l_sector, duvw_t* l_pwm_duty);
void DeadtimeCompensation(mcCpv_t* p_mpv);
void DeadtimeCompensation_BiDirectional(mcCpv_t* p_mpv);
void DeadtimeCompensation_UniDirectionalNeg(mcCpv_t* p_mpv);
void DeadtimeCompensation_UniDirectionalPos(mcCpv_t* p_mpv);
void CurrentPiController(mcCpv_t* p_mpv);
void SpeedPiController(mcCpv_t* p_mpv);
void RampController(const mcRmpCntParams_t* l_rampParams, mcsp_t l_sprefin,
    rmps_t l_rampin, rmps_t* l_ramp, mcsp_t *l_spref, freq_t l_freq);
void FieldWeakeningReset(mcfwv_t *l_fwv, mcv_t l_vamp);
mcc_t FieldWeakening(const mcfwp_t *l_fwp, mcv_t l_vamp, mcv_t l_vbus, mcfwv_t *l_fwv);


#endif
