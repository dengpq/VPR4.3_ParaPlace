#include <stdio.h>
#include <string.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "hash.h"
#include "read_place.h"


static int get_subblock(int i, int j, int block_num);
static void read_place_header(FILE* fp, char* net_file, char* arch_file,
                              char* buf);


void read_user_pad_loc(char* pad_loc_file)
{
    /* Reads in the locations of the IO_TYPE pads from a file. */
    hash_t** hash_table, *h_ptr;
    int iblk, i, j, xtmp, ytmp, block_num, isubblk;
    FILE* fp;
    char buf[BUFSIZE], bname[BUFSIZE], *ptr;
    printf("\nReading locations of IO_TYPE pads from %s.\n", pad_loc_file);
    linenum = 0;
    fp = my_fopen(pad_loc_file, "r");
    hash_table = alloc_hash_table();

    for (iblk = 0; iblk < num_blocks; ++iblk) {
        if (blocks[iblk].block_type == INPAD_TYPE
              || blocks[iblk].block_type == OUTPAD_TYPE) {
            h_ptr = insert_in_hash_table(hash_table, blocks[iblk].name, iblk);
            blocks[iblk].x = OPEN;   /* Mark as not seen yet. */
        }
    }

    for (i = 0; i <= num_grid_columns + 1; ++i) {
        for (j = 0; j <= num_grid_rows + 1; ++j) {
            if (clb_grids[i][j].grid_type == IO_TYPE) {
                for (isubblk = 0; isubblk < io_ratio; ++isubblk) {
                    clb_grids[i][j].u.io_blocks[isubblk] = OPEN;    /* Flag for err. check */
                }
            }
        }
    }

    ptr = my_fgets(buf, BUFSIZE, fp);

    while (ptr != NULL) {
        ptr = my_strtok(buf, TOKENS, fp, buf);

        if (ptr == NULL) {
            ptr = my_fgets(buf, BUFSIZE, fp);
            continue;      /* Skip blank or comment lines. */
        }

        strcpy(bname, ptr);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL) {
            printf("Error:  line %d is incomplete.\n", linenum);
            exit(1);
        }

        sscanf(ptr, "%d", &xtmp);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL) {
            printf("Error:  line %d is incomplete.\n", linenum);
            exit(1);
        }

        sscanf(ptr, "%d", &ytmp);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL) {
            printf("Error:  line %d is incomplete.\n", linenum);
            exit(1);
        }

        sscanf(ptr, "%d", &isubblk);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr != NULL) {
            printf("Error:  extra characters at end of line %d.\n", linenum);
            exit(1);
        }

        h_ptr = get_hash_entry(hash_table, bname);

        if (h_ptr == NULL) {
            printf("Error:  blocks %s on line %d: no such IO_TYPE pad.\n",
                   bname, linenum);
            exit(1);
        }

        block_num = h_ptr->index;
        i = xtmp;
        j = ytmp;

        if (blocks[block_num].x != OPEN) {
            printf("Error:  line %d.  Block %s listed twice in pad file.\n",
                   linenum, bname);
            exit(1);
        }

        if (i < 0 || i > num_grid_columns + 1 || j < 0 || j > num_grid_rows + 1) {
            printf("Error:  blocks #%d (%s) location\n", block_num, bname);
            printf("(%d,%d) is out of range.\n", i, j);
            exit(1);
        }

        blocks[block_num].x = i;   /* Will be reloaded by initial_placement anyway. */
        blocks[block_num].y = j;   /* I need to set .x only as a done flag.         */

        if (clb_grids[i][j].grid_type != IO_TYPE) {
            printf("Error:  attempt to place IO_TYPE blocks %s in \n", bname);
            printf("an illegal location (%d, %d).\n", i, j);
            exit(1);
        }

        if (isubblk >= io_ratio || isubblk < 0) {
            printf("Error:  Block %s subblock number (%d) on line %d is out of "
                   "range.\n", bname, isubblk, linenum);
            exit(1);
        }

        clb_grids[i][j].u.io_blocks[isubblk] = block_num;
        clb_grids[i][j].m_usage++;
        ptr = my_fgets(buf, BUFSIZE, fp);
    }

    for (iblk = 0; iblk < num_blocks; iblk++) {
        if ((blocks[iblk].block_type == INPAD_TYPE
              || blocks[iblk].block_type == OUTPAD_TYPE) && blocks[iblk].x == OPEN) {
            printf("Error:  IO_TYPE blocks %s location was not specified in "
                   "the pad file.\n", blocks[iblk].name);
            exit(1);
        }
    }

    for (i = 0; i <= num_grid_columns + 1; ++i) {
        for (j = 0; j <= num_grid_rows + 1; ++j) {
            if (clb_grids[i][j].grid_type == IO_TYPE) {
                for (isubblk = 0; isubblk < clb_grids[i][j].m_usage; isubblk++) {
                    if (clb_grids[i][j].u.io_blocks[isubblk] == OPEN) {
                        printf("Error:  The IO_TYPE blocks at (%d, %d) do not have \n"
                               "consecutive subblock numbers starting at 0.\n", i, j);
                        exit(1);
                    }
                }
            }
        }
    }

    fclose(fp);
    free_hash_table(hash_table);
    printf("Successfully read %s.\n\n", pad_loc_file);
}


void dump_clbs(void)
{
    /* Output routine for debugging. */
    int i, j, index;

    for (i = 0; i <= num_grid_columns + 1; ++i) {
        for (j = 0; j <= num_grid_rows + 1; ++j) {
            printf("clb (%d,%d):  type: %d  m_usage: %d\n",
                   i, j, clb_grids[i][j].grid_type,
                   clb_grids[i][j].m_usage);

            if (clb_grids[i][j].grid_type == CLB_TYPE) {
                printf("blocks: %d\n", clb_grids[i][j].u.blocks);
            }

            if (clb_grids[i][j].grid_type == IO_TYPE) {
                printf("io_blocks: ");

                for (index = 0; index < clb_grids[i][j].m_usage; index++) {
                    printf("%d  ", clb_grids[i][j].u.io_blocks[index]);
                }

                printf("\n");
            }
        }
    }

    for (i = 0; i < num_blocks; i++) {
        printf("blocks: %d, (i,j): (%d, %d)\n", i, blocks[i].x, blocks[i].y);
    }
}


void print_place(char* place_file, char* net_file, char* arch_file)
{
    /* Prints out the placement of the circuit.  The architecture and    *
     * netlist files used to generate this placement are recorded in the *
     * file to avoid loading a placement with the wrong support files    *
     * later.                                                            */
    int i, subblock;
    FILE* fp = my_fopen(place_file, "w");
    fprintf(fp, "Netlist file: %s   Architecture file: %s\n", net_file,
            arch_file);
    fprintf(fp, "Array size: %d x %d logic blocks\n\n", num_grid_columns, num_grid_rows);
    fprintf(fp, "#blocks name\tx\ty\tsubblk\tblock number\n");
    fprintf(fp, "#----------\t--\t--\t------\t------------\n");

    for (i = 0; i < num_blocks; i++) {
        fprintf(fp, "%s\t", blocks[i].name);

        if (strlen(blocks[i].name) < 8) {
            fprintf(fp, "\t");
        }

        fprintf(fp, "%d\t%d", blocks[i].x, blocks[i].y);

        if (blocks[i].block_type == CLB_TYPE) {
            fprintf(fp, "\t%d", 0);       /* Sub blocks number not meaningful. */
        } else {              /* IO_TYPE blocks.  Save sub blocks number. */
            subblock = get_subblock(blocks[i].x, blocks[i].y, i);
            fprintf(fp, "\t%d", subblock);
        }

        fprintf(fp, "\t#%d\n", i);
    }

    fclose(fp);
}


static int get_subblock(int i, int j, int block_num)
{
    /* Use this routine only for IO_TYPE blocks.  It passes back the index of the *
     * subblock containing blocks block_num at location (i,j).                     */
    int k;

    for (k = 0; k < io_ratio; k++) {
        if (clb_grids[i][j].u.io_blocks[k] == block_num) {
            return (k);
        }
    }

    printf("Error in get_subblock.  Block %d is not at location (%d,%d)\n",
           block_num, i, j);
    exit(1);
}


void parse_placement_file(char* place_file, char* net_file, char* arch_file)
{
    /* Reads the blocks in from a previously placed circuit.             */
    char bname[BUFSIZE];
    char buf[BUFSIZE], *ptr;
    hash_t** hash_table, *h_ptr;
    int i, j, block_num, isubblock, xtmp, ytmp;
    printf("Reading the placement from file %s.\n", place_file);
    FILE* fp = my_fopen(place_file, "r");
    linenum = 0;
    read_place_header(fp, net_file, arch_file, buf);

    for (i = 0; i <= num_grid_columns + 1; ++i) {
        for (j = 0; j <= num_grid_rows + 1; ++j) {
            clb_grids[i][j].m_usage = 0;

            if (clb_grids[i][j].grid_type == IO_TYPE) {
                for (isubblock = 0; isubblock < io_ratio; ++isubblock) {
                    clb_grids[i][j].u.io_blocks[isubblock] = OPEN;
                }
            }
        }
    }

    for (i = 0; i < num_blocks; i++) {
        blocks[i].x = OPEN;    /* Flag to show not read yet. */
    }

    hash_table = alloc_hash_table();

    for (i = 0; i < num_blocks; i++) {
        h_ptr = insert_in_hash_table(hash_table, blocks[i].name, i);
    }

    ptr = my_fgets(buf, BUFSIZE, fp);

    while (ptr != NULL) {
        ptr = my_strtok(buf, TOKENS, fp, buf);

        if (ptr == NULL) {
            ptr = my_fgets(buf, BUFSIZE, fp);
            continue;      /* Skip blank or comment lines. */
        }

        strcpy(bname, ptr);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL) {
            printf("Error:  line %d is incomplete.\n", linenum);
            exit(1);
        }

        sscanf(ptr, "%d", &xtmp);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL) {
            printf("Error:  line %d is incomplete.\n", linenum);
            exit(1);
        }

        sscanf(ptr, "%d", &ytmp);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL) {
            printf("Error:  line %d is incomplete.\n", linenum);
            exit(1);
        }

        sscanf(ptr, "%d", &isubblock);
        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr != NULL) {
            printf("Error:  extra characters at end of line %d.\n", linenum);
            exit(1);
        }

        h_ptr = get_hash_entry(hash_table, bname);

        if (h_ptr == NULL) {
            printf("Error:  blocks %s on line %d does not exist in the netlist.\n",
                   bname, linenum);
            exit(1);
        }

        block_num = h_ptr->index;
        i = xtmp;
        j = ytmp;

        if (blocks[block_num].x != OPEN) {
            printf("Error:  line %d.  Block %s listed twice in placement file.\n",
                   linenum, bname);
            exit(1);
        }

        if (i < 0 || i > num_grid_columns + 1 || j < 0 || j > num_grid_rows + 1) {
            printf("Error in read_place.  Block #%d (%s) location\n", block_num, bname);
            printf("(%d,%d) is out of range.\n", i, j);
            exit(1);
        }

        blocks[block_num].x = i;
        blocks[block_num].y = j;

        if (clb_grids[i][j].grid_type == CLB_TYPE) {
            if (blocks[block_num].block_type != CLB_TYPE) {
                printf("Error in read_place.  Attempt to place blocks #%d (%s) in\n",
                       block_num, bname);
                printf("a logic blocks location (%d, %d).\n", i, j);
                exit(1);
            }

            clb_grids[i][j].u.blocks = block_num;
            clb_grids[i][j].m_usage++;
        } else if (clb_grids[i][j].grid_type == IO_TYPE) {
            if (blocks[block_num].block_type != INPAD_TYPE
                  && blocks[block_num].block_type != OUTPAD_TYPE) {
                printf("Error in read_place.  Attempt to place blocks #%d (%s) in\n",
                       block_num, bname);
                printf("an IO_TYPE blocks location (%d, %d).\n", i, j);
                exit(1);
            }

            if (isubblock >= io_ratio || isubblock < 0) {
                printf("Error:  Block %s subblock number (%d) on line %d is out of "
                       "range.\n", bname, isubblock, linenum);
                exit(1);
            }

            clb_grids[i][j].u.io_blocks[isubblock] = block_num;
            clb_grids[i][j].m_usage++;
        } else {  /* Block type was ILLEGAL or some unknown value */
            printf("Error in read_place.  Block #%d (%s) is in an illegal ",
                   block_num, bname);
            printf("location.\nLocation specified: (%d,%d).\n", i, j);
            exit(1);
        }

        ptr = my_fgets(buf, BUFSIZE, fp);
    }

    free_hash_table(hash_table);
    fclose(fp);

    for (i = 0; i < num_blocks; i++) {
        if (blocks[i].x == OPEN) {
            printf("Error in read_place:  blocks %s location was not specified in "
                   "the placement file.\n", blocks[i].name);
            exit(1);
        }
    }

    for (i = 0; i <= num_grid_columns + 1; ++i) {
        for (j = 0; j <= num_grid_rows + 1; ++j) {
            if (clb_grids[i][j].grid_type == IO_TYPE) {
                for (isubblock = 0; isubblock < clb_grids[i][j].m_usage; isubblock++) {
                    if (clb_grids[i][j].u.io_blocks[isubblock] == OPEN) {
                        printf("Error:  The IO_TYPE blocks at (%d, %d) do not have \n"
                               "consecutive subblock numbers starting at 0.\n", i, j);
                        exit(1);
                    }
                }
            }
        }
    }

    printf("Successfully read %s.\n", place_file);
}


static void read_place_header(FILE* fp, char* net_file, char* arch_file,
                              char* buf)
{
    /* Reads the header from the placement file.  Used only to check that this *
     * placement file matches the current architecture, netlist, etc.          */
    char* line_one_names[] = {"Netlist", "file:", " ", "Architecture", "file:"};
    char* line_two_names[] = {"Array", "size:", " ", "x", " ", "logic", "blocks"};
    char net_check[BUFSIZE], arch_check[BUFSIZE], *ptr;
    int nx_check, ny_check, i;
    ptr = my_fgets(buf, BUFSIZE, fp);

    if (ptr == NULL) {
        printf("Error:  netlist file and architecture file used not listed.\n");
        exit(1);
    }

    ptr = my_strtok(buf, TOKENS, fp, buf);

    while (ptr == NULL) {   /* Skip blank or comment lines. */
        ptr = my_fgets(buf, BUFSIZE, fp);

        if (ptr == NULL) {
            printf("Error:  netlist file and architecture file used not listed.\n");
            exit(1);
        }

        ptr = my_strtok(buf, TOKENS, fp, buf);
    }

    for (i = 0; i <= 5; i++) {
        if (i == 2) {
            strcpy(net_check, ptr);
        } else if (i == 5) {
            strcpy(arch_check, ptr);
        } else {
            if (strcmp(ptr, line_one_names[i]) != 0) {
                printf("Error on line %d, word %d:  \n"
                       "Expected keyword %s, got %s.\n", linenum, i + 1,
                       line_one_names[i], ptr);
                exit(1);
            }
        }

        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL && i != 5) {
            printf("Error:  Unexpected end of line on line %d.\n", linenum);
            exit(1);
        }
    }

    if (strcmp(net_check, net_file) != 0) {
        printf("Warning:  Placement generated with netlist file %s:\n",
               net_check);
        printf("current net file is %s.\n", net_file);
    }

    if (strcmp(arch_check, arch_file) != 0) {
        printf("Warning:  Placement generated with architecture file %s:\n",
               arch_check);
        printf("current architecture file is %s.\n", arch_file);
    }

    /* Now check the second line (array size). */
    ptr = my_fgets(buf, BUFSIZE, fp);

    if (ptr == NULL) {
        printf("Error:  Array size not listed.\n");
        exit(1);
    }

    ptr = my_strtok(buf, TOKENS, fp, buf);

    while (ptr == NULL) {   /* Skip blank or comment lines. */
        ptr = my_fgets(buf, BUFSIZE, fp);

        if (ptr == NULL) {
            printf("Error:  array size not listed.\n");
            exit(1);
        }

        ptr = my_strtok(buf, TOKENS, fp, buf);
    }

    for (i = 0; i <= 6; i++) {
        if (i == 2) {
            sscanf(ptr, "%d", &nx_check);
        } else if (i == 4) {
            sscanf(ptr, "%d", &ny_check);
        } else {
            if (strcmp(ptr, line_two_names[i]) != 0) {
                printf("Error on line %d, word %d:  \n"
                       "Expected keyword %s, got %s.\n", linenum, i + 1,
                       line_two_names[i], ptr);
                exit(1);
            }
        }

        ptr = my_strtok(NULL, TOKENS, fp, buf);

        if (ptr == NULL && i != 6) {
            printf("Error:  Unexpected end of line on line %d.\n", linenum);
            exit(1);
        }
    }

    if (nx_check != num_grid_columns || ny_check != num_grid_rows) {
        printf("Error:  placement file assumes an array size of %d x %d.\n",
               nx_check, ny_check);
        printf("Current size is %d x %d.\n", num_grid_columns, num_grid_rows);
        exit(1);
    }
}
