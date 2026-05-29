#ifndef MCGLOBALS_H
#define MCGLOBALS_H

#include "mcinclude.h"
extern mc_t mc;
extern mcCpv_t mpv[SUPPORTED_TOTAL_MOTOR];
extern const mcCp_t mcp_default;
extern const mcCp_t* mcp_px[SUPPORTED_TOTAL_MOTOR];

extern const mcCp_t mcp_dw_circ_W20007335;
extern const mcCp_t mcp_dw_circ_W11715301;
extern const mcCp_t mcp_dw_circ_W20016965;
extern const mcCp_t mcp_dw_circ_W11652801;

extern const mcCp_t mcp_dw_drain_W11402566;
extern const mcCp_t mcp_dw_drain_W11186148;
extern const mcCp_t mcp_dw_drain_W11377410;

#endif
