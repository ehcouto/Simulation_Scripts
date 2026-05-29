#ifndef MCSTATE_H
#define MCSTATE_H


#define STATE_FLOW_INIT     ((mcstc_t)0)    
#define STATE_FLOW_FAULT    ((mcstc_t)0)
#define STATE_FLOW_BOOTSTRAP    ((mcstc_t)(FOC_DEAD_TIME_COMPENSATION))
#define STATE_FLOW_ONTHEFLY	((mcstc_t)(FOC_PARK_TRANSFORM | \
			                FOC_DCBUS_RIPPLE_ELEMINATION | FOC_SPACE_VECTOR_MODULATION | FOC_DEAD_TIME_COMPENSATION))
#define STATE_FLOW_STOPPED  ((mcstc_t)(\
                            FOC_PARK_TRANSFORM | FOC_CURRENT_CONTROLLER | \
                            FOC_INVERSE_PARK_TRANSFORM | FOC_DCBUS_RIPPLE_ELEMINATION | \
                            FOC_SPACE_VECTOR_MODULATION | FOC_DEAD_TIME_COMPENSATION))
#define STATE_FLOW_STARTUP  ((mcstc_t)(FOC_RAMP_SPEED_ESTIMATION | \
                            FOC_PARK_TRANSFORM | FOC_CURRENT_CONTROLLER | \
                            FOC_INVERSE_PARK_TRANSFORM | FOC_DCBUS_RIPPLE_ELEMINATION | \
                            FOC_SPACE_VECTOR_MODULATION | FOC_DEAD_TIME_COMPENSATION))
#define STATE_FLOW_RAMP     ((mcstc_t)(FOC_RAMP_SPEED_ESTIMATION | \
                            FOC_PARK_TRANSFORM | FOC_CURRENT_CONTROLLER | \
                            FOC_INVERSE_PARK_TRANSFORM | FOC_DCBUS_RIPPLE_ELEMINATION | \
                            FOC_SPACE_VECTOR_MODULATION | FOC_DEAD_TIME_COMPENSATION | \
                            FOC_SPEED_CONTROLLER | FOC_RAMP_CONTROLLER | FOC_FIELD_WEAKENING | FOC_MTPA))
#define STATE_FLOW_TURNOFF  ((mcstc_t)(\
                            FOC_PARK_TRANSFORM | FOC_CURRENT_CONTROLLER | FOC_INVERSE_PARK_TRANSFORM | \
                            FOC_DCBUS_RIPPLE_ELEMINATION | FOC_SPACE_VECTOR_MODULATION | \
                            FOC_DEAD_TIME_COMPENSATION))
#define STATE_FLOW_HARD_BRAKE   ((mcstc_t)(FOC_RAMP_SPEED_ESTIMATION | \
                                FOC_PARK_TRANSFORM))


void mcStateMachineHandlerSL(mcCpv_t* p_mpv);
void mcStateMachineHandlerFL(mcCpv_t* p_mpv);
void mcSetStateCommand(mcCpv_t *p_mpv);

#endif
