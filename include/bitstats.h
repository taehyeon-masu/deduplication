#ifndef BITSTATS_H
#define BITSTATS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* 블록 내 각 비트의 "변동 횟수"를 추적하기 위한 구조체
 *
 * - block_bits   : 블록 당 비트 개수 (block_bytes * 8)
 * - block_bytes  : 블록 당 바이트 수 (ceil(block_bits / 8))
 * - change_counts[bit_pos] :
 *      bit_pos 위치의 비트가 이전 블록과 다른 값을 가졌던 횟수
 * - prev_block   : 직전 블록의 raw bytes (변동 여부 비교용)
 * - has_prev     : 이전 블록 존재 여부
 */
typedef struct BitStats {
    size_t      block_bits;
    size_t      block_bytes;
    uint64_t   *change_counts;
    unsigned char *prev_block;
    int         has_prev;
} BitStats;

/* 초기화: block_bits = block_bytes * 8 같이 넘기면 됨 */
int  bitstats_init(BitStats *bs, size_t block_bits);

/* 카운터/이전 블록 정보 리셋 (block_bits는 유지) */
void bitstats_reset(BitStats *bs);

/* 메모리 해제 */
void bitstats_free(BitStats *bs);

/* 블록 하나 반영
 *  - block: raw 블록 (deviation 추출로 수정되기 "전" 데이터 사용 권장)
 *  - block_bytes: 실제 블록 바이트 수 (bs->block_bytes와 동일해야 함)
 *
 * 첫 블록은 has_prev=0 이라 change_counts는 증가하지 않고
 * prev_block만 채운다. 두 번째 블록부터 변동 여부를 센다.
 */
void bitstats_update(BitStats *bs,
                     const unsigned char *block,
                     size_t block_bytes);

/* dst에 src의 카운터를 누적 (block_bits 동일해야 함) */
int  bitstats_accumulate(BitStats *dst, const BitStats *src);

/* 통계 출력
 *  - label: "segment 0", "global" 등 구분용 태그
 *  - out  : stderr 또는 로그용 파일 포인터
 *
 * bit_pos, (byte_idx, bit_in_byte) 기준으로 변동 횟수를 모두 출력
 */
void bitstats_print(const BitStats *bs,
                    const char *label,
                    FILE *out);

#endif /* BITSTATS_H */
