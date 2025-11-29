#ifndef DICTIONARY_H
#define DICTIONARY_H

#include <stddef.h>

typedef struct {
    unsigned char **blocks;
    int size;
    int capacity;
    size_t block_size;
} Dictionary;

void dict_init(Dictionary *dict, size_t block_size);

void dict_free(Dictionary *dict);

int dict_find(const Dictionary *dict, const unsigned char *block);

int dict_add(Dictionary *dict, const unsigned char *block);

#endif