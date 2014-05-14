#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "place_parallel.h"

/************** Types and defines local to place.c ***************************/
/* New added data structures for parallel placement */
/* the grid in architecture will be partition into x_partition(columns) X  *
 * y_partition(rows). For 8-threads, I'd like to set 4x2 regions. */ 
int x_partition[64] = {
    1, 1, 1, 2, 1,   /* 1-5   */
    3, 0, 4, 3, 5,   /* 6-10  */
    0, 4, 0, 7, 5,   /* 11-15 */
    4, 0, 6, 0, 5,   /* 16-20 */
    7, 11, 0, 6, 5,  /* 21-25 */
    13, 9, 7, 0, 6,  /* 29-30 */
    7, 6, 0, 19, 7,  /* 35-40 */
    0, 7, 0, 11, 9,  /* 41-45 */
    23, 0, 8, 7, 10, /* 46-50 */
    0, 0, 0, 9, 11,  /* 51-55 */
    8, 0, 0, 0, 10,  /* 56-60 */
    0, 0, 9, 8       /* 61-64 */
};

int y_partition[64] = {
    1, 1, 1, 2, 1,   /* 1-5   */
    2, 0, 2, 3, 2,   /* 6-10  */
    0, 3, 0, 2, 3,   /* 11-15 */
    4, 0, 3, 0, 4,   /* 19-20 */ 
    3, 2, 0, 4, 5,   /* 21-25 */
    2, 3, 4, 0, 5,   /* 29-30 */
    0, 4, 3, 0, 5,   /* 31-35 */
    6, 0, 0, 0, 5,   /* 39-40 */
    0, 6, 0, 4, 5,   /* 41-45 */
    2, 0, 6, 7, 5,   /* 49-50 */
    0, 0, 0, 6, 5,   /* 51-55 */
    7, 0, 0, 0, 6,   /* 59-60 */
    0, 0, 7, 8       /* 61-64 */
};

static void update_from_global_to_local_hori(local_block_t*  local_block_ptr,
                                             grid_tile_t**  local_grid_ptr,
                                             int x_start,
                                             int x_end,
                                             int y_start,
                                             int y_end);

static void update_from_global_to_local_vert(local_block_t* local_block_ptr,
                                             grid_tile_t**  local_grid_ptr,
                                             int x_start,
                                             int x_end,
                                             int y_start,
                                             int y_end);

static void update_from_local_to_global(local_block_t* local_block,
                                        grid_tile_t**  local_grid,
                                        int x_start,
                                        int x_end,
                                        int y_start,
                                        int y_end);

