#include <stdlib.h>
#include <string.h>
#include "hash.h"
#include "util.h"

#define HASHSIZE 4093

static int hash_value(char* name);



hash_t** alloc_hash_table(void) {

    /* Creates a hash table with HASHSIZE different locations (hash values).   */

    hash_t** hash_table;

    hash_table = (hash_t**) my_calloc(sizeof(hash_t*),
                                             HASHSIZE);
    return (hash_table);
}


void free_hash_table(hash_t** hash_table)
{
    /* Frees all the storage associated with a hash table. */
    int i;
    hash_t* h_ptr, *temp_ptr;

    for (i = 0; i < HASHSIZE; i++) {
        h_ptr = hash_table[i];

        while (h_ptr != NULL) {
            free(h_ptr->name);
            temp_ptr = h_ptr->next;
            free(h_ptr);
            h_ptr = temp_ptr;
        }
    }

    free(hash_table);
}


struct s_hash_iterator start_hash_table_iterator(void) {

    /* Call this routine before you start going through all the elements in    *
     * a hash table.  It sets the internal indices to the start of the table.  */

    struct s_hash_iterator hash_iterator;

    hash_iterator.i = -1;
    hash_iterator.h_ptr = NULL;
    return (hash_iterator);
}


hash_t* get_next_hash(hash_t** hash_table, struct
                             s_hash_iterator* hash_iterator) {

    /* Returns the next occupied hash entry, and moves the iterator structure    *
     * forward so the next call gets the next entry.                             */

    int i;
    hash_t* h_ptr;

    i = hash_iterator->i;
    h_ptr = hash_iterator->h_ptr;

    while (h_ptr == NULL) {
        i++;

        if (i >= HASHSIZE) {
            return (NULL);    /* End of table */
        }

        h_ptr = hash_table[i];
    }

    hash_iterator->h_ptr = h_ptr->next;
    hash_iterator->i = i;
    return (h_ptr);
}


hash_t* insert_in_hash_table(hash_t** hash_table, char* name,
                                    int next_free_index) {

    /* Adds the string pointed to by name to the hash table, and returns the    *
     * hash structure created or updated.  If name is already in the hash table *
     * the count member of that hash element is incremented.  Otherwise a new   *
     * hash entry with a count of zero and an index of next_free_index is       *
     * created.                                                                 */

    int i;
    hash_t* h_ptr, *prev_ptr;

    i = hash_value(name);
    prev_ptr = NULL;
    h_ptr = hash_table[i];

    while (h_ptr != NULL) {
        if (strcmp(h_ptr->name, name) == 0) {
            h_ptr->count++;
            return (h_ptr);
        }

        prev_ptr = h_ptr;
        h_ptr = h_ptr->next;
    }

    /* Name string wasn't in the hash table.  Add it. */

    h_ptr = (hash_t*) my_malloc(sizeof(hash_t));

    if (prev_ptr == NULL) {
        hash_table[i] = h_ptr;
    } else {
        prev_ptr->next = h_ptr;
    }

    h_ptr->next = NULL;
    h_ptr->index = next_free_index;
    h_ptr->count = 1;
    h_ptr->name = (char*) my_malloc((strlen(name) + 1) * sizeof(char));
    strcpy(h_ptr->name, name);
    return (h_ptr);
}


hash_t* get_hash_entry(hash_t** hash_table, char* name) {

    /* Returns the hash entry with this name, or NULL if there is no            *
     * corresponding entry.                                                     */

    int i;
    hash_t* h_ptr;

    i = hash_value(name);
    h_ptr = hash_table[i];

    while (h_ptr != NULL) {
        if (strcmp(h_ptr->name, name) == 0) {
            return (h_ptr);
        }

        h_ptr = h_ptr->next;
    }

    return (NULL);
}


static int hash_value(char* name)
{
    /* Creates a hash key from a character string.  Only the first character and *
     * the last 8 characters of the string are used -- that may be dumb.         */
    int i, k;
    int val = 0, mult = 1;
    i = strlen(name);
    k = max(i - 8, 0);

    for (i = strlen(name) - 1; i >= k; i--) {
        val += mult * ((int) name[i]);
        mult *= 7;
    }

    val += (int) name[0];
    val %= HASHSIZE;
    return(val);
}
