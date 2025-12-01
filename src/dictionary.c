// src/dictionary.c
#include "../include/dictionary.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void dict_init(Dictionary *dict, size_t block_size)
{
    dict->head = NULL;
    dict->tail = NULL;
    dict->size = 0;
    dict->block_size = block_size;
}

void dict_free(Dictionary *dict)
{
    if (!dict) return;

    DictNode *cur = dict->head;
    while (cur) {
        DictNode *next = cur->next;
        free(cur->block);
        free(cur);
        cur = next;
    }
    dict->head = dict->tail = NULL;
    dict->size = 0;
    dict->block_size = 0;
}

int dict_find(const Dictionary *dict, const unsigned char *block)
{
    const DictNode *cur = dict->head;
    while (cur) {
        if (memcmp(cur->block, block, dict->block_size) == 0) {
            return cur->index;
        }
        cur = cur->next;
    }
    return -1;
}

int dict_add(Dictionary *dict, const unsigned char *block)
{
    DictNode *node = (DictNode *)malloc(sizeof(DictNode));
    if (!node) {
        fprintf(stderr, "Failed to allocate DictNode\n");
        exit(1);
    }
    node->block = (unsigned char *)malloc(dict->block_size);
    if (!node->block) {
        fprintf(stderr, "Failed to allocate block copy\n");
        free(node);
        exit(1);
    }
    memcpy(node->block, block, dict->block_size);

    node->index = dict->size;
    node->next  = NULL;

    if (dict->tail) {
        dict->tail->next = node;
    } else {
        dict->head = node;
    }
    dict->tail = node;
    dict->size += 1;

    return node->index;
}

int dict_size(const Dictionary *dict)
{
    return dict ? dict->size : 0;
}

const unsigned char *dict_get_block(const Dictionary *dict, int index)
{
    if (!dict || index < 0 || index >= dict->size) return NULL;

    const DictNode *cur = dict->head;
    while (cur) {
        if (cur->index == index) {
            return cur->block;
        }
        cur = cur->next;
    }
    return NULL;
}
