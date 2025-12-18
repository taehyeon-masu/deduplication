#include "../include/bitstats.h"

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>  /* PRIu64 */

static inline uint8_t get_bit_from_block(const unsigned char *buf, size_t bit_pos)
{
    size_t byte_idx = bit_pos / 8u;
    int    bit_idx  = (int)(bit_pos % 8u);  /* LSB-first */
    return (uint8_t)((buf[byte_idx] >> bit_idx) & 1u);
}

BitStats *bitstats_create(size_t num_bits)
{
    BitStats *bs = (BitStats *)malloc(sizeof(BitStats));
    if (!bs) return NULL;

    bs->num_bits      = num_bits;
    bs->prev_bits     = (uint8_t *)malloc(num_bits * sizeof(uint8_t));
    bs->initialized   = (uint8_t *)malloc(num_bits * sizeof(uint8_t));
    bs->change_counts = (uint64_t *)malloc(num_bits * sizeof(uint64_t));

    if (!bs->prev_bits || !bs->initialized || !bs->change_counts)
    {
        free(bs->prev_bits);
        free(bs->initialized);
        free(bs->change_counts);
        free(bs);
        return NULL;
    }

    memset(bs->prev_bits, 0, num_bits * sizeof(uint8_t));
    memset(bs->initialized, 0, num_bits * sizeof(uint8_t));
    memset(bs->change_counts, 0, num_bits * sizeof(uint64_t));

    return bs;
}

void bitstats_free(BitStats *bs)
{
    if (!bs) return;
    free(bs->prev_bits);
    free(bs->initialized);
    free(bs->change_counts);
    free(bs);
}

void bitstats_update_block(BitStats *bs,
                           const unsigned char *block,
                           size_t block_bytes)
{
    if (!bs || !block) return;
    size_t block_bits = block_bytes * 8u;
    if (block_bits < bs->num_bits)
        block_bits = bs->num_bits;  /* 방어적이지만, 보통은 == 여야 함 */

    size_t num_bits = bs->num_bits;
    if (num_bits > block_bits) num_bits = block_bits;

    for (size_t bit_pos = 0; bit_pos < num_bits; ++bit_pos)
    {
        uint8_t cur = get_bit_from_block(block, bit_pos);
        if (!bs->initialized[bit_pos])
        {
            bs->initialized[bit_pos] = 1;
            bs->prev_bits[bit_pos]   = cur;
        }
        else
        {
            if (cur != bs->prev_bits[bit_pos])
            {
                bs->change_counts[bit_pos]++;
                bs->prev_bits[bit_pos] = cur;
            }
        }
    }
}

/* 정렬용 구조체 */
typedef struct {
    size_t   bit_index;
    uint64_t count;
} BitChange;

static int cmp_bitchange_desc(const void *a, const void *b)
{
    const BitChange *pa = (const BitChange *)a;
    const BitChange *pb = (const BitChange *)b;

    if (pa->count < pb->count) return 1;   /* 내림차순 */
    if (pa->count > pb->count) return -1;
    if (pa->bit_index > pb->bit_index) return 1;
    if (pa->bit_index < pb->bit_index) return -1;
    return 0;
}

void bitstats_print_sorted(FILE *out,
                           const BitStats *bs,
                           size_t max_print)
{
    if (!bs || !out) return;

    size_t num_bits = bs->num_bits;
    BitChange *arr = (BitChange *)malloc(num_bits * sizeof(BitChange));
    if (!arr) return;

    for (size_t i = 0; i < num_bits; ++i)
    {
        arr[i].bit_index = i;
        arr[i].count     = bs->change_counts[i];
    }

    qsort(arr, num_bits, sizeof(BitChange), cmp_bitchange_desc);

    fprintf(out, "=== Bit change counts (sorted, desc) ===\n");
    size_t limit = (max_print == 0 || max_print > num_bits) ? num_bits : max_print;
    for (size_t rank = 0; rank < limit; ++rank)
    {
        size_t   bit_pos = arr[rank].bit_index;
        uint64_t cnt     = arr[rank].count;
        size_t  byte_idx = bit_pos / 8u;
        int     bit_in   = (int)(bit_pos % 8u);

        fprintf(out,
                "  rank %3zu: bit %3zu (byte %3zu, bit %d) => %" PRIu64 " changes\n",
                rank, bit_pos, byte_idx, bit_in, cnt);
    }

    free(arr);
}
