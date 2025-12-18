#ifndef BITSTATS_H
#define BITSTATS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* ------------------------------------------------------------
 * BitStats
 *  - num_bits: 블록 당 비트 개수 (block_bytes * 8)
 *  - prev_bits[i]: 직전 블록에서의 비트값 (0/1)
 *  - initialized[i]: prev_bits 초기화 여부
 *  - change_counts[i]: 블록 간에서 값이 바뀐 횟수
 * ------------------------------------------------------------ */
typedef struct BitStats {
    size_t   num_bits;
    uint8_t *prev_bits;
    uint8_t *initialized;
    uint64_t *change_counts;
} BitStats;

BitStats *bitstats_create(size_t num_bits);
void      bitstats_free(BitStats *bs);

/* block_bytes는 실제 블록 바이트 수 (num_bits == block_bytes * 8 가정) */
void bitstats_update_block(BitStats *bs,
                           const unsigned char *block,
                           size_t block_bytes);

/* 변동 횟수 기준으로 내림차순 정렬해서 상위 max_print개 출력
 * max_print == 0이면 전체 출력
 */
void bitstats_print_sorted(FILE *out,
                           const BitStats *bs,
                           size_t max_print);

#endif /* BITSTATS_H */
