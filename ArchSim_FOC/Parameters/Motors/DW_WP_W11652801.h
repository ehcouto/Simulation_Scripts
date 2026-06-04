#include "mcglobals.h"

#define MOTOR_TEMP 25.0f

#define RS_25C     24.9f
#define RS         (RS_25C * (1.0f + 0.00393f * (MOTOR_TEMP - 25.0f))) // Resistance at MOTOR_TEMP, considering temperature coefficient of copper/Aluminum 
#define LD         0.079f
#define LQ         0.079f   
#define LAMBDA_M   0.0753f
#define INERTIA_J  8.7E-6f
#define VISC_B     4.6E-5f
#define POLE_PAIRS 3U
#define TL_MAX     0.216f
#define MAX_SPEED  3500.0f

#define MOTOR_RAMP_RPMS  750.0f //rpm

void set_motor(uint32_t motor_id)
{
    mcp_px[motor_id] = &mcp_dw_circ_W11652801;
    fsp_px[motor_id] = &fsp_dw_circ_W11652801;
}