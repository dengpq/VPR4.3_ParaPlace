#ifndef CHECK_ROUTE_H
#define CHECK_ROUTE_H

#include "vpr_types.h"

void check_route(enum e_route_type route_type, int num_switch,
                 t_ivec** clb_opins_used_locally);

#endif

