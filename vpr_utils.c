#include "vpr_types.h"
#include "globals.h"
#include "vpr_utils.h"

/* This module contains subroutines that are used in several unrelated parts *
 * of VPR.  They are VPR-specific utility routines.                          */

/******************** Subroutine definitions ********************************/
boolean is_opin(int ipin)
{
    /* Returns TRUE if this clb pin is an output, FALSE otherwise. */
    int iclass = clb_pin_class[ipin];
    if (class_inf[iclass].type == DRIVER) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/* load the clb fanout information(or the number of nets where it connections) *
 * for each clb. Attention, the most important parameters were: int*           *
 * num_uses_of_clb_ipin and int* num_uses_of_sblk_opin.                        */
void load_one_clb_fanout_count(int subblock_lut_size,
                               subblock_t* subblock_inf,
                               int  num_subblocks, /* num_of_subblocks_in_block */
                               int* num_uses_of_clb_ipin,
                               int* num_uses_of_sblk_opin,
                               int iclb)
{
    int isub = -1;
    for (isub = 0; isub < num_subblocks; ++isub) {
    /* Is the subblock output connected to a CLB_TYPE opin that actually goes *
     * somewhere?  Necessary to check that the CLB_TYPE opin connects to      *
     * something because some logic blocks result in netlists where      *
     * subblock outputs being automatically hooked to a CLB_TYPE opin under   *
     * all conditions.                                                   */

        /* First deal with the subblock OUTPUT pin */
        int output_pin = subblock_inf[isub].output;
        if (output_pin != OPEN) { /* OPEN, DRIVER, RECEIVER */
            if (blocks[iclb].nets[output_pin] != OPEN) { /* CLB_TYPE output is used */
                ++num_uses_of_sblk_opin[isub];
            }
        }

        /* Then deal with the subblock's INPUT pins(lut-k) */
        int ipin = -1;
        for (ipin = 0; ipin < subblock_lut_size; ++ipin) {
            int input_pin = subblock_inf[isub].inputs[ipin];
            if (input_pin != OPEN && input_pin < pins_per_clb) { /* Driven by CLB_TYPE ipin */
                ++num_uses_of_clb_ipin[input_pin];
            } else if (input_pin != OPEN && input_pin >= pins_per_clb) {
                /* Driven by sblk output in same clb(sharing signals) *
                 * This pin must connect more than 1 net.             */
                ++num_uses_of_sblk_opin[input_pin - pins_per_clb];
            } else {
                ; /* No Operations */
            }
        }  /* End for each sblk ipin */

        /* Last deal with the subblocks' CLOCK input pin */
        int clock_pin = subblock_inf[isub].clock;
        if (clock_pin != OPEN && clock_pin < pins_per_clb) { /* Driven by CLB_TYPE ipin */
            ++num_uses_of_clb_ipin[clock_pin];
        } else if (clock_pin != OPEN && clock_pin >= pins_per_clb) {
            /* Driven by sblk output in same clb(sharing signals) *
             * This pin must connect more than 1 net. */
            ++num_uses_of_sblk_opin[clock_pin - pins_per_clb];
        } else {
            ; /* No operations */
        }
    }  /* End for each subblock */
} /* end of void load_one_clb_fanout_count() */


placer_costs_t*  init_placer_costs(void)
{
    placer_costs_t* placer_costs_ptr = (placer_costs_t*)malloc(sizeof(placer_costs_t));

    placer_costs_ptr->m_bb_cost = placer_costs_ptr->m_timing_cost = 0.0;
    placer_costs_ptr->m_delay_cost = placer_costs_ptr->m_total_cost = 0.0;

    placer_costs_ptr->m_av_cost = placer_costs_ptr->m_av_bb_cost = 0.0;
    placer_costs_ptr->m_av_timing_cost = placer_costs_ptr->m_av_delay_cost = 0.0;

    placer_costs_ptr->m_new_bb_cost = placer_costs_ptr->m_new_timing_cost = placer_costs_ptr->m_delay_cost = 0.0;

    placer_costs_ptr->m_inverse_prev_bb_cost = placer_costs_ptr->m_inverse_prev_timing_cost = 0.0;

    return placer_costs_ptr;
}

placer_paras_t*  init_placer_paras(void)
{
    placer_paras_t* placer_paras = (placer_paras_t*)malloc(sizeof(placer_paras_t));
    placer_paras->m_fixed_pins = FALSE;
    placer_paras->m_sum_of_squares = placer_paras->m_place_delay_value = 0.0;
    placer_paras->m_width_factor = placer_paras->m_num_connections = 0;

    placer_paras->m_max_delay = placer_paras->m_crit_exponent = 0.0;

    placer_paras->m_inner_crit_iter_count = placer_paras->m_outer_crit_iter_count = 0;
    placer_paras->m_move_limit = placer_paras->m_inner_recompute_limit = 0;

    placer_paras->m_temper = 0.0;  /* sucess_ratio */
    placer_paras->m_range_limit = placer_paras->m_final_rlim = placer_paras->m_inverse_delta_rlim = 1.0;

    placer_paras->m_total_iter = placer_paras->m_success_sum = 0;
    placer_paras->m_success_ratio = placer_paras->m_std_dev = 0.0;

    return placer_paras;
}

void  free_placer_costs(placer_costs_t*  placer_costs)
{
    free(placer_costs);
    placer_costs = NULL;
}

void  free_placer_paras(placer_paras_t*  placer_paras)
{
    free(placer_paras);
    placer_paras = NULL;
}

