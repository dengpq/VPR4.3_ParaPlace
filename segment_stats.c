#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "segment_stats.h"


/*************** Variables and defines local to this module ****************/

#define LONGLINE 0



/******************* Subroutine definitions ********************************/


void get_segment_usage_stats(int num_segment, segment_info_t* segment_inf)
{
    /* Computes statistics on the fractional utilization of segments by type    *
     * (index) and by length.  This routine needs a valid rr_graph, and a       *
     * completed routing.  Note that segments cut off by the end of the array   *
     * are counted as full-length segments (e.g. length 4 even if the last 2    *
     * units of wire were chopped off by the chip edge).                        */
    int ivex, length, seg_type, max_segment_length, cost_index;
    int* seg_occ_by_length, *seg_cap_by_length;   /* [0..max_segment_length] */
    int* seg_occ_by_type, *seg_cap_by_type;       /* [0..num_segment-1]      */
    double utilization;
    max_segment_length = 0;

    for (seg_type = 0; seg_type < num_segment; seg_type++) {
        if (segment_inf[seg_type].longline == FALSE)
            max_segment_length = max(max_segment_length,
                                     segment_inf[seg_type].length);
    }

    seg_occ_by_length = (int*) my_calloc((max_segment_length + 1),
                                         sizeof(int));
    seg_cap_by_length = (int*) my_calloc((max_segment_length + 1),
                                         sizeof(int));
    seg_occ_by_type = (int*) my_calloc(num_segment, sizeof(int));
    seg_cap_by_type = (int*) my_calloc(num_segment, sizeof(int));

    for (ivex = 0; ivex < num_rr_nodes; ivex++) {
        if (rr_node[ivex].type == CHANX || rr_node[ivex].type == CHANY) {
            cost_index = rr_node[ivex].cost_index;
            seg_type = rr_indexed_data[cost_index].seg_index;

            if (!segment_inf[seg_type].longline) {
                length = segment_inf[seg_type].length;
            } else {
                length = LONGLINE;
            }

            seg_occ_by_length[length] += rr_node[ivex].m_usage;
            seg_cap_by_length[length] += rr_node[ivex].capacity;
            seg_occ_by_type[seg_type] += rr_node[ivex].m_usage;
            seg_cap_by_type[seg_type] += rr_node[ivex].capacity;
        }
    }

    printf("\nSegment usage by type (index):\n");
    printf("Segment type       Fractional utilization\n");
    printf("------------       ----------------------\n");

    for (seg_type = 0; seg_type < num_segment; seg_type++) {
        if (seg_cap_by_type[seg_type] != 0) {
            utilization = (double) seg_occ_by_type[seg_type] /
                          (double) seg_cap_by_type[seg_type];
            printf("%8d                  %5.3g\n", seg_type, utilization);
        }
    }

    printf("\nSegment usage by length:\n");
    printf("Segment length       Fractional utilization\n");
    printf("--------------       ----------------------\n");

    for (length = 1; length <= max_segment_length; length++) {
        if (seg_cap_by_length[length] != 0) {
            utilization = (double) seg_occ_by_length[length] /
                          (double) seg_cap_by_length[length];
            printf("%9d                   %5.3g\n", length, utilization);
        }
    }

    if (seg_cap_by_length[LONGLINE] != 0) {
        utilization = (double) seg_occ_by_length[LONGLINE] /
                      (double) seg_cap_by_length[LONGLINE];
        printf("   longline                 %5.3g\n", utilization);
    }

    free(seg_occ_by_length);
    free(seg_cap_by_length);
    free(seg_occ_by_type);
    free(seg_cap_by_type);
}
