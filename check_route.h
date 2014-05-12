#ifndef CHECK_ROUTE_H
#define CHECK_ROUTE_H

#include "vpr_types.h"

void check_route(router_types_t route_type, int num_switch,
                 vector_t** clb_opins_used_locally);

#endif

