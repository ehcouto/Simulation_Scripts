#include "mcinclude.h"


static void mcBootstrapStateHandlerSL(mcCpv_t *p_mpv);
static void mcStartupStateHandlerSL(mcCpv_t *p_mpv);
static void mcRampStateHandlerSL(mcCpv_t *p_mpv);
static void mcTurnoffStateHandlerSL(mcCpv_t *p_mpv);
static void mcTurnoffStateHandlerFL(mcCpv_t *p_mpv);
static void mcStoppedStateHandlerSL(mcCpv_t *p_mpv);
static void mcFaultStateHandlerSL(mcCpv_t *p_mpv);
static void mcOnTheFlyStateHandlerSL(mcCpv_t *p_mpv);
static void mcOnTheFlyStateHandlerFL(mcCpv_t *p_mpv);


void mcStateMachineHandlerSL(mcCpv_t* p_mpv)
{
    mc_bool_t encDataExistState;
    mcsp_t l_sprefin;
    
    /********** apply mechanic speed request to motor control process *********/
    if((p_mpv->v.sprefin == (mcsp_t)0) && (p_mpv->v.sprefmec != (mcsp_t)0))
    {
        p_mpv->rotRequestCntr++;
    }
    else
    {
        ;   /* do nothing */
    }
    
    l_sprefin = p_mpv->v.sprefmec * p_mpv->p.bratio;
    if(l_sprefin > p_mpv->p.sprefmax)
    {
        p_mpv->v.sprefin = p_mpv->p.sprefmax;
    }
    else
    {
        p_mpv->v.sprefin = l_sprefin;
    }
    p_mpv->v.sprefin_abs = mathAbsoluteFlt(p_mpv->v.sprefin);
    /**************************************************************************/
    
    /*************** Get encoder position if encoder is used ******************/
    encDataExistState = p_mpv->func->mcBrdGetEncoderData(&p_mpv->v.thenc.th,&p_mpv->v.spenc);
    
    if((p_mpv->v.sprefin != (mcsp_t)0) && (encDataExistState == MC_TRUE))
    {
        p_mpv->v.thenc.cos = (mctri_t)(mathCosFlt(p_mpv->v.thenc.th));
        p_mpv->v.thenc.sin = (mctri_t)(mathSinFlt(p_mpv->v.thenc.th));
    }
    else
    {
        ;   /* do nothing */
    }
    /**************************************************************************/
    
    if(mathAbsoluteFlt(p_mpv->v.sprefin) > mathAbsoluteFlt(p_mpv->v.spref))
    {
        if(p_mpv->v.sprefin * p_mpv->v.spref >= 0)
        {
        p_mpv->v.acc = ACC_POS;
    }
        else
        {
            p_mpv->v.acc = ACC_NEG;
        }
    }
    else if(mathAbsoluteFlt(p_mpv->v.sprefin) < mathAbsoluteFlt(p_mpv->v.spref))
    {
        if(p_mpv->v.sprefin * p_mpv->v.spref >= 0)
        {
        p_mpv->v.acc = ACC_NEG;
    }
    else
    {
            p_mpv->v.acc = ACC_POS;
        }
    }
    else
    {
        p_mpv->v.acc = ACC_NO;
    }
    
    if(p_mpv->v.state == JUST_POWERED)
    {
        if(p_mpv->v.just_powered_delay < p_mpv->p.just_powered_min_duration)
        {
            p_mpv->v.just_powered_delay++;
        }
        else
        {
            p_mpv->v.state = RESET_DRIVE;
        }
    }
    else if(p_mpv->v.state == RESET_DRIVE)
    {
        p_mpv->func->mcBrdPwmDisable();
        p_mpv->v.pwm_enabled = MC_FALSE;

        if(p_mpv->control_state == MC_CONTROL_STATE_ACTIVE)
        {
            mcVariablesInit(p_mpv);
            mcObserversInit(p_mpv);

            p_mpv->func->mcBrdCurrentCalibrationInit();            
            p_mpv->v.pstate = MC_TRUE;
            p_mpv->v.state = INIT;
    	}
        else
        {
            ;   /* do nothing */
        }
    }
    else if (p_mpv->v.state == INIT)
    {
        if(p_mpv->v.sprefmec != (mcsp_t)0)
        {
            if(p_mpv->v.calready == MC_TRUE)
            {
                mc.func.mcDrvTurnOnInrushRelay();
            
                p_mpv->func->mcBrdPwmEnable();
                p_mpv->v.pwm_enabled = MC_TRUE;
                
                mcBootstrapStart(p_mpv);
                
                p_mpv->v.state = BOOTSTRAP;
            }
            else
            {
                ;   /* do nothing */
            }
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else if(p_mpv->v.state == BOOTSTRAP)
    {
        mcBootstrapStateHandlerSL(p_mpv);
        
        if(p_mpv->v.btstrp.step == BOOTSTRAP_APPLIED)
        {
            p_mpv->v.spref = (mcsp_t)0;
            
            p_mpv->v.state = ON_THE_FLY;
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else if(p_mpv->v.state == ON_THE_FLY)
    {
        if(p_mpv->v.sprefin == (mcsp_t)0)
        {
            if(p_mpv->p.disPwmWhenStop == MC_TRUE)
            {
                p_mpv->func->mcBrdPwmDisable();
                p_mpv->v.pwm_enabled = MC_FALSE;
            }	    
            p_mpv->v.state = TURNOFF;
        }
        else
        {
    	    mcOnTheFlyStateHandlerSL(p_mpv);

            if(p_mpv->v.mcOnTheFlyStartup.active == MC_FALSE)
                p_mpv->v.state = STOPPED;	    
        }
    }
    else if (p_mpv->v.state == STOPPED)
    {
        mcStoppedStateHandlerSL(p_mpv);
        
        if(p_mpv->v.sprefin != (mcsp_t)0)
        {
            p_mpv->v.startup.ol.state = STARTUP_OL_STATE_PASSIVE;
            p_mpv->v.turnoff.ol.state = TURNOFF_OL_STATE_PASSIVE;
            p_mpv->v.state = STARTUP;
            p_mpv->v.transready = MC_FALSE;

            if(p_mpv->v.sprefin > (mcsp_t)0)
            {
                p_mpv->v.rot = ROT_CW;
            }
            else
            {
                p_mpv->v.rot = ROT_CCW;
            }
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else if(p_mpv->v.state == STARTUP)
    {
        if(p_mpv->v.sprefin == (mcsp_t)0)
        {
	    if(p_mpv->p.disPwmWhenStop == MC_TRUE)
	    {
	        p_mpv->func->mcBrdPwmDisable();
	        p_mpv->v.pwm_enabled = MC_FALSE;
	    }
	    
            p_mpv->v.state = TURNOFF;
        }
        else
        {
            mcStartupStateHandlerSL(p_mpv);
            
            if (p_mpv->v.transready == MC_TRUE)
            {
                p_mpv->v.state = RAMP;
            }
            else
            {
                ;   /* do nothing */
            }
        }
        
        if(p_mpv->v.state != STARTUP)
        {
            p_mpv->v.startup.ol.state = STARTUP_OL_STATE_PASSIVE;
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else if (p_mpv->v.state == RAMP)
    {
        mcRampStateHandlerSL(p_mpv);
        
        if((mathAbsoluteFlt(p_mpv->v.spref) < p_mpv->p.minsp_c2o && p_mpv->v.acc != ACC_POS) || 
		((p_mpv->v.sprefin == 0.0f) && (p_mpv->p.disPwmWhenStop == MC_TRUE)))
        {
            if(p_mpv->p.disPwmWhenStop == MC_TRUE)
            {
	        p_mpv->func->mcBrdPwmDisable();
	        p_mpv->v.pwm_enabled = MC_FALSE;
            }
            
            p_mpv->v.state = TURNOFF;
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else if (p_mpv->v.state == TURNOFF)
    {
        mcTurnoffStateHandlerSL(p_mpv);
    }
    else if(p_mpv->v.state == FAULT)
    {
        mcFaultStateHandlerSL(p_mpv);

        p_mpv->v.state = RESET_DRIVE;
        p_mpv->v.pstate = MC_FALSE;
    }
    else
    {
        ;  /* do nothing */
    }
}

void mcStateMachineHandlerFL(mcCpv_t* p_mpv)
{
    if((p_mpv->v.state == STARTUP) && (p_mpv->p.startup.method == METHOD_OPEN_LOOP))
    {
        p_mpv->v.thref.th = mathAngleSumDeg(p_mpv->v.thref.th, 
                                            (mcsp_t)MC_TWOPI2DEG * p_mpv->v.spref * (mcdeg_t)p_mpv->p.phys.pp / (mcsp_t)RPM2RPS_DIVIDER / (mcsp_t)p_mpv->p.freqFL);
        mathCalcSinCosFlt(p_mpv->v.thref.th, &p_mpv->v.thref.sin, &p_mpv->v.thref.cos);
    }
    else if(p_mpv->v.state == TURNOFF)
    {
        mcTurnoffStateHandlerFL(p_mpv);
    }
    else if(p_mpv->v.state == INIT)
    {
        p_mpv->v.calready = p_mpv->func->mcBrdCurrentCalibrationHandler();
    }
    else if((p_mpv->v.state == ON_THE_FLY) && (p_mpv->v.mcOnTheFlyStartup.active == MC_TRUE))
    {    
        mcOnTheFlyStateHandlerFL(p_mpv);
    }
    else
    {
        ;  /* do nothing */
    }
    p_mpv->v.spactmec = p_mpv->v.spact / p_mpv->p.bratio;

	torqCompensationHandlerFL(p_mpv);
}

void mcSetStateCommand(mcCpv_t *p_mpv)
{
    if (p_mpv->v.state == BOOTSTRAP)
    {
        p_mpv->v.statecom = STATE_FLOW_BOOTSTRAP;
    }
    else if (p_mpv->v.state == STOPPED)
    {
        p_mpv->v.statecom = STATE_FLOW_STOPPED;
    }
    else if (p_mpv->v.state == STARTUP)
    {
            p_mpv->v.statecom = STATE_FLOW_STARTUP;
        }
    else if (p_mpv->v.state == RAMP)
    {
        if(p_mpv->v.hwHardBrake.state == MC_HW_HB_STATE_ACTIVE)
        {
            p_mpv->v.statecom = STATE_FLOW_HARD_BRAKE;
        }
        else
        {
            p_mpv->v.statecom = STATE_FLOW_RAMP;
        }
    }
    else if (p_mpv->v.state == TURNOFF)
    {
        p_mpv->v.statecom = STATE_FLOW_TURNOFF;
    }
    else if (p_mpv->v.state == FAULT)
    {
        p_mpv->v.statecom = STATE_FLOW_FAULT;
    }
    else if (p_mpv->v.state == INIT)
    {
        p_mpv->v.statecom = STATE_FLOW_INIT;
    }
    else if(p_mpv->v.state == ON_THE_FLY)
    {
        p_mpv->v.statecom = STATE_FLOW_ONTHEFLY;
    }
    else
    {
        ;  /* do nothing */
    }
}

static void mcBootstrapStateHandlerSL(mcCpv_t *p_mpv)
{
    if(p_mpv->v.btstrp.step == BOOTSTRAP_INITIATED)
    {
        p_mpv->v.btstrp.timer = 0u;
        p_mpv->v.btstrp.step = BOOTSTRAP_FOR_PHASE_U;
        p_mpv->func->mcBrdPwmEnablePhaseU();
    }    
    else if((p_mpv->v.btstrp.step == BOOTSTRAP_FOR_PHASE_U) && 
            (p_mpv->v.btstrp.timer >= p_mpv->p.btstrp.duration))
    {
        p_mpv->v.btstrp.timer = 0u;
        p_mpv->v.btstrp.step = BOOTSTRAP_FOR_PHASE_V;
        p_mpv->func->mcBrdPwmEnablePhaseV();
    }    
    else if((p_mpv->v.btstrp.step == BOOTSTRAP_FOR_PHASE_V) && 
            (p_mpv->v.btstrp.timer >= p_mpv->p.btstrp.duration))
    {
        p_mpv->v.btstrp.timer = 0u;
        p_mpv->v.btstrp.step = BOOTSTRAP_FOR_PHASE_W;
        p_mpv->func->mcBrdPwmEnablePhaseW();
    }    
    else if((p_mpv->v.btstrp.step == BOOTSTRAP_FOR_PHASE_W) && 
            (p_mpv->v.btstrp.timer >= p_mpv->p.btstrp.duration))
    {
        p_mpv->v.btstrp.timer = 0u;
        p_mpv->func->mcBrdPwmEnablePhaseUVW();
        p_mpv->v.btstrp.step = BOOTSTRAP_APPLIED;
    }
    else
    {
        ;  /* do nothing */
    }
    
    p_mpv->v.btstrp.timer++;
}

static void mcStartupStateHandlerSL(mcCpv_t *p_mpv)
{
    if(p_mpv->p.startup.method == METHOD_OPEN_LOOP)
    {
        if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_PASSIVE)
        {
            p_mpv->v.startup.ol.statePrev = p_mpv->v.startup.ol.state;
            p_mpv->v.startup.ol.state = STARTUP_OL_STATE_TH_ALIGN_0;
	    }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_TH_ALIGN_0)
        {
              if((msec_t)p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.thalign_duration)
              {
                  p_mpv->v.startup.ol.state = STARTUP_OL_STATE_TH_ALIGN_1ST;
              }        
              else if(p_mpv->v.startup.ol.timer == 0u)
              {
                  p_mpv->v.spref = (mcsp_t)0;
                  if(p_mpv->p.startup.ol.thref_thalign != 0.0f)
                  {
                      p_mpv->v.thref.th = p_mpv->p.startup.ol.thref_thalign;
                  }
                  else
                  {
                      ;         /* do nothing */
                  }
                  p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_thalign_0.d;
                  p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_thalign_0.q;
              }
              else
              {
                  ;         /* do nothing */
              }
        }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_TH_ALIGN_1ST)
        {
              if((msec_t)p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.thalign_duration)
              {
                  p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_RCALC_1ST_COMPLETED;
                  p_mpv->v.startup.ol.state = STARTUP_OL_STATE_TH_ALIGN_2ND;
              }
              else if(p_mpv->v.startup.ol.timer == 0u)
              {
                  p_mpv->v.spref = (mcsp_t)0;
                  p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_thalign_1st.d;
                  p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_thalign_1st.q;
                  
                  p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_RCALC_1ST_WAIT;
              }
              else
              {
                  if(p_mpv->v.idq.d >= p_mpv->v.idqref.d && 
                     p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.thermalRefCurrentLatency)
                  {
                      p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_RCALC_1ST_ACTIVE;
                  }
                  else
                  {
                      ;   /* do nothing */
                  }
              }
        }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_TH_ALIGN_2ND)
        {
              if((msec_t)p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.thalign_duration)
              {
                  p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_RCALC_2ND_COMPLETED;                  
                  p_mpv->v.startup.ol.state = STARTUP_OL_STATE_MEC_ALIGN_1ST;
              }
              else if(p_mpv->v.startup.ol.timer == 0u)
              {
                  p_mpv->v.spref = (mcsp_t)0;
                  p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_thalign_2nd.d;
                  p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_thalign_2nd.q;
                  
                  p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_RCALC_2ND_WAIT;
              }
              else
              {
                  if(p_mpv->v.idq.d >= p_mpv->v.idqref.d && 
                     p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.thermalRefCurrentLatency)
                  {
                      p_mpv->v.fs_state.thermal_state = FS_THERMAL_STATE_RCALC_2ND_ACTIVE;
                  }
                  else
                  {
                      ;   /* do nothing */
                  }
              }
        }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_MEC_ALIGN_1ST)
        {
              if((msec_t)p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.mecalign1st_duration)
              {
                  p_mpv->v.startup.ol.state = STARTUP_OL_STATE_MEC_ALIGN_2ND;
              }
              else if(p_mpv->v.startup.ol.timer == 0u)
              {
                  p_mpv->v.spref = (mcsp_t)0;
                  if(p_mpv->p.startup.ol.thref_mecalign != 0.0f)
                  {
                      p_mpv->v.thref.th = p_mpv->p.startup.ol.thref_mecalign;
                  }
                  else
                  {
                      ;         /* do nothing */
                  }
              }
              else
              { // apply current ramp to defined align current
                  if (p_mpv->v.idqref.d < p_mpv->p.startup.ol.idq_mecalign1st.d)
                  {
                      p_mpv->v.idqref.d += mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign1st.d);                      
                      if(p_mpv->v.idqref.d > p_mpv->p.startup.ol.idq_mecalign1st.d)
                      {
                          p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign1st.d;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else if(p_mpv->v.idqref.d > p_mpv->p.startup.ol.idq_mecalign1st.d)
                  {
                      p_mpv->v.idqref.d -= mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign1st.d);
                      if(p_mpv->v.idqref.d < p_mpv->p.startup.ol.idq_mecalign1st.d)
                      {
                          p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign1st.d;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else
                  {
                      p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign1st.d;
                  }

                  if (p_mpv->v.idqref.q < p_mpv->p.startup.ol.idq_mecalign1st.q)
                  {
                      p_mpv->v.idqref.q += mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign1st.q);
                      if(p_mpv->v.idqref.q > p_mpv->p.startup.ol.idq_mecalign1st.q)
                      {
                          p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign1st.q;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else if(p_mpv->v.idqref.q > p_mpv->p.startup.ol.idq_mecalign1st.q)
                  {
                      p_mpv->v.idqref.q -= mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign1st.q);
                      if(p_mpv->v.idqref.q < p_mpv->p.startup.ol.idq_mecalign1st.q)
                      {
                          p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign1st.q;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else
                  {
                      p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign1st.q;
                  }
              }
        }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_MEC_ALIGN_2ND)
        {
              if((msec_t)p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.mecalign2nd_duration)
              {
                  p_mpv->v.startup.ol.state = STARTUP_OL_STATE_MEC_ALIGN_3RD;
              }
              else if(p_mpv->v.startup.ol.timer == 0u)
              {
                  p_mpv->v.spref = (mcsp_t)0;
              }
              else
              { // apply current ramp to defined align current
                  if (p_mpv->v.idqref.d < p_mpv->p.startup.ol.idq_mecalign2nd.d)
                  {
                      p_mpv->v.idqref.d += mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign2nd.d);
                      if(p_mpv->v.idqref.d > p_mpv->p.startup.ol.idq_mecalign2nd.d)
                      {
                          p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign2nd.d;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else if(p_mpv->v.idqref.d > p_mpv->p.startup.ol.idq_mecalign2nd.d)
                  {
                      p_mpv->v.idqref.d -= mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign2nd.d);
                      if(p_mpv->v.idqref.d < p_mpv->p.startup.ol.idq_mecalign2nd.d)
                      {
                          p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign2nd.d;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else
                  {
                      p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign2nd.d;
                  }

                  if (p_mpv->v.idqref.q < p_mpv->p.startup.ol.idq_mecalign2nd.q)
                  {
                      p_mpv->v.idqref.q += mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign2nd.q);
                      if(p_mpv->v.idqref.q > p_mpv->p.startup.ol.idq_mecalign2nd.q)
                      {
                          p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign2nd.q;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else if(p_mpv->v.idqref.q > p_mpv->p.startup.ol.idq_mecalign2nd.q)
                  {
                      p_mpv->v.idqref.q -= mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign2nd.q);
                      if(p_mpv->v.idqref.q < p_mpv->p.startup.ol.idq_mecalign2nd.q)
                      {
                          p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign2nd.q;
                      }
                      else
                      {
                          ;     /* do nothing */
                      }
                  }
                  else
                  {
                      p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign2nd.q;
                  }
              }
        }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_MEC_ALIGN_3RD)
        {
        	if((msec_t)p_mpv->v.startup.ol.timer >= p_mpv->p.startup.ol.mecalign3rd_duration)
            {
        		p_mpv->v.startup.ol.state = STARTUP_OL_STATE_RAMPUP;
            }
            else if(p_mpv->v.startup.ol.timer == 0u)
            {
                p_mpv->v.spref = (mcsp_t)0;
            }
            else
            { // apply current ramp to defined align current
            	if (p_mpv->v.idqref.d < p_mpv->p.startup.ol.idq_mecalign3rd.d)
                {
            		p_mpv->v.idqref.d += mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign3rd.d);
                    if(p_mpv->v.idqref.d > p_mpv->p.startup.ol.idq_mecalign3rd.d)
                    {
                    	p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign3rd.d;
                    }
                    else
                    {
                        ;     /* do nothing */
                    }
                }
                else if(p_mpv->v.idqref.d > p_mpv->p.startup.ol.idq_mecalign3rd.d)
                {
                	p_mpv->v.idqref.d -= mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign3rd.d);
                    if(p_mpv->v.idqref.d < p_mpv->p.startup.ol.idq_mecalign3rd.d)
                    {
                    	p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign3rd.d;
                    }
                    else
                    {
                        ;     /* do nothing */
                    }
                }
                else
                {
                	p_mpv->v.idqref.d = p_mpv->p.startup.ol.idq_mecalign3rd.d;
                }

                if (p_mpv->v.idqref.q < p_mpv->p.startup.ol.idq_mecalign3rd.q)
                {
                	p_mpv->v.idqref.q += mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign3rd.q);
                    if(p_mpv->v.idqref.q > p_mpv->p.startup.ol.idq_mecalign3rd.q)
                    {
                    	p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign3rd.q;
                    }
                    else
                    {
                        ;     /* do nothing */
                    }
                }
                else if(p_mpv->v.idqref.q > p_mpv->p.startup.ol.idq_mecalign3rd.q)
                {
                	p_mpv->v.idqref.q -= mathAbsoluteFlt(p_mpv->p.startup.ol.idqramp_mecalign3rd.q);
                    if(p_mpv->v.idqref.q < p_mpv->p.startup.ol.idq_mecalign3rd.q)
                    {
                    	p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign3rd.q;
                    }
                    else
                    {
                        ;     /* do nothing */
                    }
                }
                else
                {
                    p_mpv->v.idqref.q = p_mpv->p.startup.ol.idq_mecalign3rd.q;
                }
            }
        }
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_RAMPUP)
        {
            if(p_mpv->v.startup.ol.timer == 0u)
            {
                p_mpv->v.idqref.d = p_mpv->p.startup.ol.idqmin.d;

                if(p_mpv->v.sprefin > (mcsp_t)0)
                {
                    p_mpv->v.idqref.q = p_mpv->p.startup.ol.idqmin.q;
                }
                else if(p_mpv->v.sprefin < (mcsp_t)0)
                {
                    p_mpv->v.idqref.q = -p_mpv->p.startup.ol.idqmin.q;
                    if(p_mpv->v.idqref.q != (mcc_t)0)
                    {                        
                        p_mpv->v.thref.th = mathAngleSumDeg(p_mpv->v.thref.th, MC_PI2DEG);
                    }
                }
                else
                {
                    ;  /* do nothing */
                }

                p_mpv->v.spref = (mcsp_t)0;
            }
            else
            {
                if (p_mpv->v.idqref.d < p_mpv->p.startup.ol.idqmax.d)
                {
                    p_mpv->v.idqref.d += p_mpv->p.startup.ol.idramp;
                }
                else
                {
                    p_mpv->v.idqref.d = p_mpv->p.startup.ol.idqmax.d;
                }
               
                if(p_mpv->v.sprefin > (mcsp_t)0)
                {                    
                    if (p_mpv->v.idqref.q < p_mpv->p.startup.ol.idqmax.q)
                    {
                        p_mpv->v.idqref.q += p_mpv->p.startup.ol.iqramp;
                    }
                    else
                    {
                        p_mpv->v.idqref.q = p_mpv->p.startup.ol.idqmax.q;
                    }

                    if(p_mpv->v.spref >= p_mpv->v.sprefin)
                    {
                        p_mpv->v.spref = p_mpv->v.sprefin;
                    }
                    else if(p_mpv->v.spref >= p_mpv->p.startup.ol.sprefmax)
                    {
                        p_mpv->v.spref = p_mpv->p.startup.ol.sprefmax;
                    }
                    else
                    {
                        p_mpv->v.spref += p_mpv->p.startup.ol.spreframp;
                    }
                }
                else if(p_mpv->v.sprefin < (mcsp_t)0)
                {
                    if (p_mpv->v.idqref.q > -p_mpv->p.startup.ol.idqmax.q)
                    {
                        p_mpv->v.idqref.q -= p_mpv->p.startup.ol.iqramp;
                    }
                    else
                    {
                        p_mpv->v.idqref.q = -p_mpv->p.startup.ol.idqmax.q;
                    }

                    if(p_mpv->v.spref <= p_mpv->v.sprefin)
                    {
                        p_mpv->v.spref = p_mpv->v.sprefin;
                    }
                    else if(p_mpv->v.spref <= -p_mpv->p.startup.ol.sprefmax)
                    {
                        p_mpv->v.spref = -p_mpv->p.startup.ol.sprefmax;
                    }
                    else
                    {
                        p_mpv->v.spref -= p_mpv->p.startup.ol.spreframp;
                    }
                }
                else
                {
                    ;  /* do nothing */
                }
                
		if((p_mpv->v.spref_abs > p_mpv->p.startup.ol.minValidSp) && 
                   (p_mpv->v.sperr_abs < p_mpv->p.startup.ol.spgapmax))
                {
                    p_mpv->v.startup.ol.tValidSpCnt++;
                    if(p_mpv->v.startup.ol.tValidSpCnt >= p_mpv->p.startup.ol.tValidSp)
                    {
                        p_mpv->v.startup.ol.tValidSpCnt = p_mpv->p.startup.ol.tValidSp;
                        
                        if((p_mpv->v.therr >= p_mpv->p.transition.thErrMin) && (p_mpv->v.therr <= p_mpv->p.transition.thErrMax))
                        {
				            p_mpv->v.startup.ol.state = STARTUP_OL_STATE_TRANSITION;
                        }
                        else
                        {
                            ;  /* do nothing */
                        }
                    }
                    else
                    {
                        ;   /* do nothing */
                    }
                }
                else
                {
                    p_mpv->v.startup.ol.tValidSpCnt = 0u;
                }
            }
        }   
        else if(p_mpv->v.startup.ol.state == STARTUP_OL_STATE_TRANSITION)
        {             
            if(p_mpv->p.transition.mode == TRANSITION_MODE_ANGLE_SWITCHING)
            {
                p_mpv->v.cc_manipulate.state = 1;
            }
            else if(p_mpv->p.transition.mode == TRANSITION_MODE_DIRECT)
            {
                        ;  /* do nothing */
                    }
                else
                {
                ;  /* do nothing */
            }
            
            p_mpv->v.transition.ramp = mathAbsoluteFlt(p_mpv->v.idqref.d / (mcc_t)p_mpv->p.freqSL / p_mpv->p.transition.duration);
                p_mpv->v.transready = MC_TRUE;
        }
        else
        {
            ;  /* do nothing */
        }
        
        if(p_mpv->v.startup.ol.state != p_mpv->v.startup.ol.statePrev)
        {
            p_mpv->v.startup.ol.timer = 0u;
        }
        else 
        {
            p_mpv->v.startup.ol.timer++;
        }
        p_mpv->v.startup.ol.statePrev = p_mpv->v.startup.ol.state;
    }
    else if(p_mpv->p.startup.method == METHOD_CLOSED_LOOP)
    {
        ;  /* do nothing */
    }
    else
    {
        ;  /* do nothing */
    }
}

static void mcRampStateHandlerSL(mcCpv_t *p_mpv)
{
    mcc_t l_idref;

    if(p_mpv->v.sprefin_prev != p_mpv->v.sprefin)
    {
        if(p_mpv->v.ramp_duration != (rmps_t)0.0f)
            p_mpv->v.rampin = (p_mpv->v.sprefin - p_mpv->v.spref) / p_mpv->v.ramp_duration;
        p_mpv->v.sprefin_prev = p_mpv->v.sprefin;
    }
    else
    {
        ;   /* do nothing */
    }
    
    if(p_mpv->v.fwv.idref == (mcc_t)0 && p_mpv->v.mtpa.idref == (mcc_t)0 )
    {
        if(p_mpv->v.idqref.d > (mcc_t)0)
        {
            l_idref = p_mpv->v.idqref.d - p_mpv->v.transition.ramp;
            if(l_idref < (mcc_t)0)
                l_idref = (mcc_t)0;
            p_mpv->v.idqref.d = l_idref;
        }
        else if(p_mpv->v.idqref.d < (mcc_t)0)
        {
            l_idref = p_mpv->v.idqref.d + p_mpv->v.transition.ramp;
            if(l_idref > (mcc_t)0)
                l_idref = (mcc_t)0;
            p_mpv->v.idqref.d = l_idref;
        }
        else //if(p_mpv->v.idqref.d == (mcc_t)0)
        {
            //p_mpv->v.idqref.d = 0.0f;
        }
    }
    else
    {
        ;   /* do nothing */
    }
    
    /* decide speed pi controller region and parameters */ 
    if(p_mpv->v.piSpeedOverwrite != 0)
    {
        p_mpv->v.spControllerRegion = MC_SP_CONTROLLER_REGION_0;    
    }
    else if(p_mpv->v.spref_abs < p_mpv->p.piSpeedR1toR2)
    {
        if(p_mpv->v.spControllerRegion != MC_SP_CONTROLLER_REGION_1)
        {
            p_mpv->v.spControllerRegion = MC_SP_CONTROLLER_REGION_1;
            
            p_mpv->v.piSpeed.kp = p_mpv->p.piSpeedR1.kp;
            p_mpv->v.piSpeed.ki = p_mpv->p.piSpeedR1.ki / (mcc_t)p_mpv->p.freqSL;
            p_mpv->v.piSpeed.antiWindupCoef = p_mpv->p.piSpeedR1.antiWindupCoef;
            p_mpv->v.piSpeed.antiWindupLimit = p_mpv->p.piSpeedR1.antiWindupLimit;
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else if(p_mpv->v.spref_abs < p_mpv->p.piSpeedR2toR3)
    {
        if(p_mpv->v.spControllerRegion != MC_SP_CONTROLLER_REGION_2)
        {
            p_mpv->v.spControllerRegion = MC_SP_CONTROLLER_REGION_2;
            
            p_mpv->v.piSpeed.kp = p_mpv->p.piSpeedR2.kp;
            p_mpv->v.piSpeed.ki = p_mpv->p.piSpeedR2.ki / (mcc_t)p_mpv->p.freqSL;
            p_mpv->v.piSpeed.antiWindupCoef = p_mpv->p.piSpeedR2.antiWindupCoef;
            p_mpv->v.piSpeed.antiWindupLimit = p_mpv->p.piSpeedR2.antiWindupLimit;
        }
        else
        {
            ;   /* do nothing */
        }
    }
    else
    {
        if(p_mpv->v.spControllerRegion != MC_SP_CONTROLLER_REGION_3)
        {
            p_mpv->v.spControllerRegion = MC_SP_CONTROLLER_REGION_3;
            
            p_mpv->v.piSpeed.kp = p_mpv->p.piSpeedR3.kp;
            p_mpv->v.piSpeed.ki = p_mpv->p.piSpeedR3.ki / (mcc_t)p_mpv->p.freqSL;
            p_mpv->v.piSpeed.antiWindupCoef = p_mpv->p.piSpeedR3.antiWindupCoef;
            p_mpv->v.piSpeed.antiWindupLimit = p_mpv->p.piSpeedR3.antiWindupLimit;
        }
        else
        {
            ;   /* do nothing */
        }
    }
    
    mcHwHardBrakeHandlerSL(p_mpv);
}

static void mcTurnoffStateHandlerSL(mcCpv_t *p_mpv)
{
    if(p_mpv->p.turnoff.method == METHOD_DISABLE_PWM)
    {
	if((msec_t)p_mpv->v.turnoff.ol.timer >= p_mpv->p.turnoff.disPwmLatency)
	{
	    p_mpv->v.turnoff.ol.timer = (msec_t)0.0f;
	    p_mpv->v.state = RESET_DRIVE;
	}
	else 
	{
	    p_mpv->v.spref = (mcsp_t)0.0f;
	    p_mpv->v.turnoff.ol.timer++;
	}
    }
    else if(p_mpv->v.sprefin != (mcsp_t)0)
    {
	if(p_mpv->v.pwm_enabled == MC_TRUE)
	{
            p_mpv->v.spref = (mcsp_t)0.0f;
            p_mpv->v.state = STOPPED;
	}
	else
	{
            p_mpv->v.state = RESET_DRIVE;
	}
    }
    else if(p_mpv->p.turnoff.method == METHOD_OPEN_LOOP)
    {
        if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_STATE_PASSIVE)
        {
            p_mpv->v.turnoff.ol.statePrev = TURNOFF_OL_STATE_PASSIVE;
            p_mpv->v.turnoff.ol.state = TURNOFF_OL_ROTATING;
        }        
        else if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_ROTATING)
        {
            if(p_mpv->v.spref > (mcsp_t)0)
            {
                p_mpv->v.spref -= p_mpv->p.turnoff.ol.spreframp;
                if(p_mpv->v.spref < (mcsp_t)0)
                {
                    p_mpv->v.spref = (mcsp_t)0;
                }
            }
            else if(p_mpv->v.spref < (mcsp_t)0)
            {
                p_mpv->v.spref += p_mpv->p.turnoff.ol.spreframp;
                if(p_mpv->v.spref > (mcsp_t)0)
                {
                    p_mpv->v.spref = (mcsp_t)0;
                }
            }
            else
            {
                ;  /* do nothing */
            }
            
            if((p_mpv->v.spref == (mcsp_t)0) || ((msec_t)p_mpv->v.turnoff.ol.timer >= p_mpv->p.turnoff.ol.t_rot))
            {
                p_mpv->v.spref = (mcsp_t)0;
                p_mpv->v.turnoff.ol.state = TURNOFF_OL_CURRENT_STATE1;
            }
            else if(p_mpv->v.turnoff.ol.timer == 0u)
            {
                p_mpv->v.idqref.d = p_mpv->p.turnoff.ol.idq_rot.d;
                p_mpv->v.idqref.q = p_mpv->p.turnoff.ol.idq_rot.q;
            }
            else
            {
                ;  /* do nothing */
            }
        }
        else if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_CURRENT_STATE1)
        {
              if(p_mpv->v.turnoff.ol.timer >= (htck_t)p_mpv->p.turnoff.ol.t1)
              {
                  p_mpv->v.turnoff.ol.state = TURNOFF_OL_CURRENT_STATE2;
              }
              else if(p_mpv->v.turnoff.ol.timer == 0u)
              {
                  p_mpv->v.idqref.d = p_mpv->p.turnoff.ol.idq1.d;
                  p_mpv->v.idqref.q = p_mpv->p.turnoff.ol.idq1.q;
              }
              else
              {
                  ;  /* do nothing */
              }
        }
        else if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_CURRENT_STATE2)
        {
              if(p_mpv->v.turnoff.ol.timer >= (htck_t)p_mpv->p.turnoff.ol.t2)
              {
                  p_mpv->v.turnoff.ol.state = TURNOFF_OL_CURRENT_STATE3;
              }
              else if(p_mpv->v.turnoff.ol.timer == 0u)
              {
                  p_mpv->v.idqref.d = p_mpv->p.turnoff.ol.idq2.d;
                  p_mpv->v.idqref.q = p_mpv->p.turnoff.ol.idq2.q;
              }
              else
              {
                  ;  /* do nothing */
              }
        }
        else if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_CURRENT_STATE3)
        {
              if(p_mpv->v.turnoff.ol.timer >= (htck_t)p_mpv->p.turnoff.ol.t3)
              {
                  p_mpv->v.turnoff.ol.state = TURNOFF_OL_PWM_STATE_SELECTION;
              }
              else if(p_mpv->v.turnoff.ol.timer == 0u)
              {
                  p_mpv->v.idqref.d = p_mpv->p.turnoff.ol.idq3.d;
                  p_mpv->v.idqref.q = p_mpv->p.turnoff.ol.idq3.q;
              }
              else
              {
                  ;  /* do nothing */
              }
        }
        else if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_PWM_STATE_SELECTION)
        {
            if(p_mpv->p.turnoff.ol.dispwm == MC_TRUE)
            {
                //mc.func.mcDrvTurnOffInrushRelay();
                
                p_mpv->func->mcBrdPwmDisable();
                p_mpv->v.pwm_enabled = MC_FALSE;

                p_mpv->v.state = RESET_DRIVE;
            }
            else
            {
                p_mpv->v.state = STOPPED;
            }
            p_mpv->v.turnoff.ol.state = TURNOFF_OL_STATE_PASSIVE;
        }
        else
        {
            ;  /* do nothing */
        }
        
        if (p_mpv->v.turnoff.ol.state != p_mpv->v.turnoff.ol.statePrev)
        {
            p_mpv->v.turnoff.ol.timer = 0u;
        }
        else
        {
            p_mpv->v.turnoff.ol.timer++;
        }
        p_mpv->v.turnoff.ol.statePrev = p_mpv->v.turnoff.ol.state;
    }
    else if(p_mpv->p.startup.method == METHOD_CLOSED_LOOP)
    {
        ;  /* do nothing */
    }
    else
    {
        ;  /* do nothing */
    }

    if(p_mpv->v.state != TURNOFF)
    {
	p_mpv->v.turnoff.ol.timer = 0;
	
        p_mpv->v.turnoff.ol.state = TURNOFF_OL_STATE_PASSIVE;
    }
    else
    {
        ;  /* do nothing */
    }
}

static void mcTurnoffStateHandlerFL(mcCpv_t *p_mpv)
{
    if(p_mpv->v.turnoff.ol.state == TURNOFF_OL_ROTATING)
    {
        p_mpv->v.thref.th = mathAngleSumDeg(p_mpv->v.thref.th, 
                                            (mcsp_t)MC_TWOPI2DEG * p_mpv->v.spref * (mcdeg_t)p_mpv->p.phys.pp / (mcsp_t)RPM2RPS_DIVIDER / (mcsp_t)p_mpv->p.freqFL);
        mathCalcSinCosFlt(p_mpv->v.thref.th, &p_mpv->v.thref.sin, &p_mpv->v.thref.cos);
    }
}

static void mcStoppedStateHandlerSL(mcCpv_t *p_mpv)
{
    p_mpv->v.spref = (mcsp_t)0;
    
    if (p_mpv->v.rot == ROT_CW)
    {
        p_mpv->v.idqref = p_mpv->p.stopped_current;
    }
    else if(p_mpv->v.rot == ROT_CCW)
    {
        p_mpv->v.idqref.d = p_mpv->p.stopped_current.d;
        p_mpv->v.idqref.q = -p_mpv->p.stopped_current.q;
    }
    else
    {
        ;  /* do nothing */
    }
    
    p_mpv->v.rot = ROT_NO;
}

static void mcFaultStateHandlerSL(mcCpv_t *p_mpv)
{
    p_mpv->v.pwm_enabled = MC_FALSE;

    p_mpv->v.idqref.d = (mcc_t)0;
    p_mpv->v.idqref.q = (mcc_t)0;
    
    p_mpv->v.spest = (mcsp_t)0;

    p_mpv->v.spref = (mcsp_t)0;
    p_mpv->v.spref_abs = (mcsp_t)0;
    
    p_mpv->v.sprefin = (mcsp_t)0;
    p_mpv->v.sprefin_abs = (mcsp_t)0;
    
    p_mpv->v.sprefmec = (mcsp_t)0;
    
    p_mpv->v.vdq.d = (mcv_t)0;
    p_mpv->v.vdq.q = (mcv_t)0;
    
    p_mpv->v.piDCurrent.integ = (mcv_t)0;
    p_mpv->v.piQCurrent.integ = (mcv_t)0;
}

mc_float_t onTheflyStartupTh = 0.0f;

static void mcOnTheFlyStateHandlerSL(mcCpv_t *p_mpv)
{
    if(p_mpv->p.mcOnTheFlyStartup.enabled == MC_FALSE)
    {
        p_mpv->v.mcOnTheFlyStartup.active = MC_FALSE;
    	
        return;
    }
    else if(p_mpv->v.sprefmec != 0.0f)
    {
        if(p_mpv->v.mcOnTheFlyStartup.active == MC_FALSE)
        {
            FilterMedianInit(&p_mpv->v.mcOnTheFlyStartup.sMed, 0);
            FilterAverageInit(&p_mpv->v.mcOnTheFlyStartup.sAve, 8);

            p_mpv->v.thref.th = onTheflyStartupTh;
            p_mpv->v.thref.cos = mathCosFlt(onTheflyStartupTh);
            p_mpv->v.thref.sin = mathSinFlt(onTheflyStartupTh);

            p_mpv->v.mcOnTheFlyStartup.vabInject.a = p_mpv->p.mcOnTheFlyStartup.vabFirst.a;
            p_mpv->v.mcOnTheFlyStartup.vabInject.b = p_mpv->p.mcOnTheFlyStartup.vabFirst.b;

            p_mpv->v.mcOnTheFlyStartup.iAveInteg = 0.0f;
            p_mpv->v.mcOnTheFlyStartup.integCntr = 0;

            p_mpv->v.mcOnTheFlyStartup.active = MC_TRUE;
        }
        else
        {
            p_mpv->v.mcOnTheFlyStartup.integCntr++;
            if(p_mpv->v.mcOnTheFlyStartup.integCntr > p_mpv->p.freqSL)
            {
                p_mpv->v.mcOnTheFlyStartup.iAveInteg -= p_mpv->v.mcOnTheFlyStartup.iAveIntegPerSec / p_mpv->p.freqSL;
                p_mpv->v.mcOnTheFlyStartup.integCntr = p_mpv->p.freqSL;
            }
            p_mpv->v.mcOnTheFlyStartup.iAveInteg += mathAbsoluteFlt(p_mpv->v.mcOnTheFlyStartup.iAve);
            p_mpv->v.mcOnTheFlyStartup.iAveIntegPerSec = p_mpv->v.mcOnTheFlyStartup.iAveInteg;
            
            if(p_mpv->v.mcOnTheFlyStartup.integCntr >= p_mpv->p.freqSL && p_mpv->v.mcOnTheFlyStartup.iAveIntegPerSec < p_mpv->p.mcOnTheFlyStartup.iAveIntegPerSecMin)
            {
                p_mpv->v.idqref = p_mpv->v.idq;
                ParkTransform(&p_mpv->v.vab, &p_mpv->v.thref, &p_mpv->v.vdq);
                p_mpv->v.piDCurrent.integ = p_mpv->v.vdq.d;
                p_mpv->v.piQCurrent.integ = p_mpv->v.vdq.q;

                p_mpv->v.mcOnTheFlyStartup.integCntr = 0;

                p_mpv->v.mcOnTheFlyStartup.active = MC_FALSE;

                return;
            }
            else
            {
                if(p_mpv->v.mcOnTheFlyStartup.vabInject.a < p_mpv->p.mcOnTheFlyStartup.vabMax.a)
                    p_mpv->v.mcOnTheFlyStartup.vabInject.a += p_mpv->p.mcOnTheFlyStartup.vabRamp.a;
                else
                    p_mpv->v.mcOnTheFlyStartup.vabInject.a = p_mpv->p.mcOnTheFlyStartup.vabMax.a;

                if(p_mpv->v.mcOnTheFlyStartup.vabInject.b < p_mpv->p.mcOnTheFlyStartup.vabMax.b)
                    p_mpv->v.mcOnTheFlyStartup.vabInject.b += p_mpv->p.mcOnTheFlyStartup.vabRamp.b;
                else
                    p_mpv->v.mcOnTheFlyStartup.vabInject.b = p_mpv->p.mcOnTheFlyStartup.vabMax.b;
            }

            p_mpv->v.vab.a = p_mpv->v.mcOnTheFlyStartup.vabInject.a;
            p_mpv->v.vab.b = p_mpv->v.mcOnTheFlyStartup.vabInject.b;    	
        }
    }

    return;
}

static void mcOnTheFlyStateHandlerFL(mcCpv_t *p_mpv)
{
    // sampled current must be vertical to injected voltage (etc : p_mpv->v.iab.b is selected)
    FilterMedianCalc(&p_mpv->v.mcOnTheFlyStartup.sMed, p_mpv->v.iab.b, &p_mpv->v.mcOnTheFlyStartup.iMed);
    p_mpv->v.mcOnTheFlyStartup.iAve = FilterAverageCalc(&p_mpv->v.mcOnTheFlyStartup.sAve, p_mpv->v.mcOnTheFlyStartup.iMed);
	
    return;
}
