#ifndef HASH_H
#define HASH_H

typedef struct s_hash_t {
    char* name;
    int   index;
    int   count;
    struct s_hash_t* next;
} hash_t;

/* name:  The string referred to by this hash entry.                        *
 * index: The integer identifier for this entry.                            *
 * count: Number of times an element with this name has been inserted into  *
 *        the table.                                                        *
 * next:  A pointer to the next (string,index) entry that mapped to the     *
 *        same hash value, or NULL if there are no more entries.            */
struct s_hash_iterator {
    int i;
    hash_t* h_ptr;
};

/* i:  current "line" of the hash table.  That is, hash_table[i] is the     *
 *     start of the hash linked list for this hash value.                   *
 * h_ptr:  Pointer to the next hash structure to be examined in the         *
 *         iteration.                                                       */


hash_t** alloc_hash_table(void);
void   free_hash_table(hash_t** hash_table);
struct s_hash_iterator start_hash_table_iterator(void);
hash_t* get_next_hash(hash_t** hash_table, struct
                      s_hash_iterator* hash_iterator);
hash_t* insert_in_hash_table(hash_t** hash_table, char* name,
                             int next_free_index);
hash_t* get_hash_entry(hash_t** hash_table, char* name);

#endif

