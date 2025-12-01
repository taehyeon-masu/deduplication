#ifndef DICTIONARY_H
#define DICTIONARY_H

#include <stddef.h>

// 단일 연결 리스트 노드
typedef struct DictNode
{
    unsigned char *block; // block_size 바이트
    int index;            // 0,1,2,... 순서
    struct DictNode *next;
} DictNode;

typedef struct
{
    DictNode *head;
    DictNode *tail;
    int size;          // 노드 개수
    size_t block_size; // 블록 하나의 바이트 수
} Dictionary;

// 초기화
void dict_init(Dictionary *dict, size_t block_size);

// 메모리 해제
void dict_free(Dictionary *dict);

// block과 완전히 일치하는 노드를 찾고 index 반환, 없으면 -1
int dict_find(const Dictionary *dict, const unsigned char *block);

// block을 새로 추가하고 index 반환
int dict_add(Dictionary *dict, const unsigned char *block);

// 현재 사전 크기(블록 개수)
int dict_size(const Dictionary *dict);

// index(0..size-1)번째 블록 포인터 얻기
// 존재하지 않으면 NULL
const unsigned char *dict_get_block(const Dictionary *dict, int index);

#endif // DICTIONARY_H
