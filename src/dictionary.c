#include "../include/dictionary.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define INITIAL_CAPACITY 16

void dict_init(Dictionary *dict, size_t block_size)
{
    dict->size = 0;
    dict->capacity = INITIAL_CAPACITY;
    dict->block_size = block_size;
    dict->blocks = (unsigned char **)malloc(sizeof(unsigned char *) * dict->capacity);
    if (!dict->blocks)
    {
        fprintf(stderr, "Failed to allocate memory for dictionary\n");
        exit(1);
    }
}

void dict_free(Dictionary *dict)
{
    if (!dict)
        return;
    for (int i = 0; i < dict->size; ++i)
    {
        free(dict->blocks[i]);
    }
    free(dict->blocks);
    dict->blocks = NULL;
    dict->size = 0;
    dict->capacity = 0;
    dict->block_size = 0;
}

static void dict_grow(Dictionary *dict)
{
    int new_cap = dict->capacity * 2;
    unsigned char **new_blocks = (unsigned char **)realloc(dict->blocks, sizeof(unsigned char *) * new_cap);
    if (!new_blocks)
    {
        fprintf(stderr, "Failed to reallocate dictionary\n");
        exit(1);
    }
    dict->blocks = new_blocks;
    dict->capacity = new_cap;
}

int dict_find(const Dictionary *dict, const unsigned char *block)
{
    for (int i = 0; i < dict->size; ++i)
    {
        if (memcmp(dict->blocks[i], block, dict->block_size) == 0)
        {
            return i;
        }
    }
    return -1;
}

int dict_add(Dictionary *dict, const unsigned char *block)
{
    if (dict->size == dict->capacity)
    {
        dict_grow(dict);
    }
    unsigned char *copy = (unsigned char *)malloc(dict->block_size);
    if (!copy)
    {
        fprintf(stderr, "Failed to allocate block copy\n");
        exit(1);
    }
    memcpy(copy, block, dict->block_size);
    dict->blocks[dict->size] = copy;
    int idx = dict->size;
    dict->size += 1;
    return idx;
}