#include "../include/bitstats.h"

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

/* 내부 helper: LSB-first bit 접근 */
static inline uint8_t get_bit_from_bytes(const unsigned char *buf, size_t bit_pos)
{
    size_t byte_idx = bit_pos / 8u;
    int    bit_idx  = (int)(bit_pos % 8u);
    return (uint8_t)((buf[byte_idx] >> bit_idx) & 1u);
}

/* ------------------------------------------------------------
 * 초기화 / 해제
 * ------------------------------------------------------------ */

int bitstats_init(BitStats *bs, size_t num_bits)
{
    if (!bs || num_bits == 0)
        return -1;

    bs->num_bits = num_bits;
    bs->change_counts = (uint64_t *)calloc(num_bits, sizeof(uint64_t));
    bs->last_values   = (uint8_t  *)malloc(num_bits);
    if (!bs->change_counts || !bs->last_values)
    {
        free(bs->change_counts);
        free(bs->last_values);
        memset(bs, 0, sizeof(*bs));
        return -1;
    }

    memset(bs->last_values, 0, num_bits);
    bs->initialized = 0;
    return 0;
}

void bitstats_free(BitStats *bs)
{
    if (!bs) return;
    free(bs->change_counts);
    free(bs->last_values);
    memset(bs, 0, sizeof(*bs));
}

/* ------------------------------------------------------------
 * 블록 단위 업데이트
 *   - 첫 블록: last_values만 채우고, change_counts는 증가시키지 않음
 *   - 두 번째 블록부터:
 *       bit 값이 이전(last_values)와 다르면 change_counts[bit_pos]++
 *       그리고 last_values 갱신
 * ------------------------------------------------------------ */

void bitstats_update_block(BitStats *bs,
                           const unsigned char *block,
                           size_t block_bytes)
{
    if (!bs || !block) return;

    size_t block_bits = block_bytes * 8u;
    if (block_bits < bs->num_bits)
        block_bits = bs->num_bits; /* 혹시 block_bytes*8 < num_bits면 num_bits까지만 사용 */

    size_t limit = bs->num_bits;

    if (!bs->initialized)
    {
        /* 첫 블록: 기준값만 저장 */
        for (size_t i = 0; i < limit; ++i)
        {
            if (i >= block_bytes * 8u) break;
            uint8_t bit = get_bit_from_bytes(block, i);
            bs->last_values[i] = bit;
        }
        bs->initialized = 1;
        return;
    }

    /* 두 번째 이후 블록: 변화 감지 */
    for (size_t i = 0; i < limit; ++i)
    {
        if (i >= block_bytes * 8u) break;
        uint8_t bit = get_bit_from_bytes(block, i);
        if (bit != bs->last_values[i])
        {
            bs->change_counts[i] += 1;
            bs->last_values[i] = bit;
        }
    }
}

/* ------------------------------------------------------------
 * 출력: 원래 순서 (0..num_bits-1)
 * ------------------------------------------------------------ */

void bitstats_print(FILE *out, const BitStats *bs)
{
    if (!out || !bs || !bs->change_counts) return;

    fprintf(out, "Bit change counts (unsorted):\n");
    for (size_t i = 0; i < bs->num_bits; ++i)
    {
        size_t   bit_pos  = i;
        size_t   byte_idx = bit_pos / 8u;
        int      bit_in   = (int)(bit_pos % 8u);
        uint64_t cnt      = bs->change_counts[bit_pos];

        fprintf(out,
                "  bit %3zu (byte %3zu, bit %d): %" PRIu64 "\n",
                bit_pos, byte_idx, bit_in, cnt);
    }
}

/* ------------------------------------------------------------
 * 정렬된 출력: change_counts 기준 내림차순
 * ------------------------------------------------------------ */

typedef struct {
    size_t   bit_pos;
    uint64_t count;
} BitCount;

static int cmp_bitcount_desc(const void *a, const void *b)
{
    const BitCount *x = (const BitCount *)a;
    const BitCount *y = (const BitCount *)b;
    if (x->count < y->count) return 1;   /* 내림차순 */
    if (x->count > y->count) return -1;
    return 0;
}

void bitstats_print_sorted(FILE *out, const BitStats *bs)
{
    if (!out || !bs || !bs->change_counts) return;

    size_t num_bits = bs->num_bits;
    BitCount *arr = (BitCount *)malloc(sizeof(BitCount) * num_bits);
    if (!arr)
    {
        fprintf(out, "bitstats_print_sorted: malloc failed\n");
        return;
    }

    for (size_t i = 0; i < num_bits; ++i)
    {
        arr[i].bit_pos = i;
        arr[i].count   = bs->change_counts[i];
    }

    qsort(arr, num_bits, sizeof(BitCount), cmp_bitcount_desc);

    fprintf(out, "Bit change counts (sorted by count desc):\n");
    for (size_t k = 0; k < num_bits; ++k)
    {
        size_t   bit_pos  = arr[k].bit_pos;
        size_t   byte_idx = bit_pos / 8u;
        int      bit_in   = (int)(bit_pos % 8u);
        uint64_t cnt      = arr[k].count;

        fprintf(out,
                "  bit %3zu (byte %3zu, bit %d): %" PRIu64 "\n",
                bit_pos, byte_idx, bit_in, cnt);
    }

    free(arr);
}
