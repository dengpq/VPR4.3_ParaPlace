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
    /* Is the subblock output connected to a CLB opin that actually goes *
     * somewhere?  Necessary to check that the CLB opin connects to      *
     * something because some logic blocks result in netlists where      *
     * subblock outputs being automatically hooked to a CLB opin under   *
     * all conditions.                                                   */

        /* First deal with the subblock OUTPUT pin */
        int output_pin = subblock_inf[isub].output;
        if (output_pin != OPEN) { /* OPEN, DRIVER, RECEIVER */
            if (blocks[iclb].nets[output_pin] != OPEN) { /* CLB output is used */
                ++num_uses_of_sblk_opin[isub];
            }
        }

        /* Then deal with the subblock's INPUT pins(lut-k) */
        int ipin = -1;
        for (ipin = 0; ipin < subblock_lut_size; ++ipin) {
            int input_pin = subblock_inf[isub].inputs[ipin];
            if (input_pin != OPEN && input_pin < pins_per_clb) { /* Driven by CLB ipin */
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
        if (clock_pin != OPEN && clock_pin < pins_per_clb) { /* Driven by CLB ipin */
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

