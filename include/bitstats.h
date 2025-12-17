#ifndef BITSTATS_H
#define BITSTATS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* 블록 단위 비트 변동 횟수 기록용 구조체 */
typedef struct {
    size_t    num_bits;       /* 블록당 비트 개수 (block_bytes * 8) */
    uint64_t *change_counts;  /* 각 bit_pos마다 변동 횟수 */
    uint8_t  *last_values;    /* 직전 블록에서의 bit 값(0/1) */
    int       initialized;    /* 첫 블록 처리 여부 */
} BitStats;

/* 초기화: num_bits 만큼 change_counts/last_values 할당 */
int  bitstats_init(BitStats *bs, size_t num_bits);

/* 메모리 해제 */
void bitstats_free(BitStats *bs);

/* 블록 하나를 넣어서 비트 변동 횟수 갱신 */
void bitstats_update_block(BitStats *bs,
                           const unsigned char *block,
                           size_t block_bytes);

/* (옵션) 그냥 순서대로 출력 */
void bitstats_print(FILE *out, const BitStats *bs);

/* 변동 횟수 기준 내림차순 정렬해서 출력 */
void bitstats_print_sorted(FILE *out, const BitStats *bs);

#endif /* BITSTATS_H */
