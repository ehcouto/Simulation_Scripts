#include "mcglobals.h"

#define MOTOR_TEMP 25.0f

#define RS_25C     65.0f
#define RS         (RS_25C * (1.0f + 0.00393f * (MOTOR_TEMP - 25.0f))) // Resistance at MOTOR_TEMP, considering temperature coefficient of copper/Aluminum    
#define LD         0.13f
#define LQ         0.13f   
#define LAMBDA_M   0.06614f
#define INERTIA_J  2.3E-6f
#define VISC_B     1.5E-5f
#define POLE_PAIRS 1U
#define TL_MAX     0.05f
#define MAX_SPEED  3500.0f

#define MOTOR_RAMP_RPMS  2000.0f //rpm

void set_motor(uint32_t motor_id)
{
    mcp_px[motor_id] = &mcp_dw_drain_W11402566;
    fsp_px[motor_id] = &fsp_dw_drain_W11402566;
}

#define LOAD_TYPE     LOAD_TYPE_HYDRAULIC_SIMPLE
#define K_HYD         TL_MAX /(MAX_SPEED*MAX_SPEED)

//Simulation Parameters (Locally changing only for Simulation)
#define PARAMETERS_STARTUP_OL_THALIGN_DURATION       33.0f