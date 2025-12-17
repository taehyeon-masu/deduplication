#include "../include/bitstats.h"

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

int bitstats_init(BitStats *bs, size_t block_bits)
{
    if (!bs || block_bits == 0)
        return -1;

    size_t block_bytes = (block_bits + 7u) / 8u;

    bs->block_bits  = block_bits;
    bs->block_bytes = block_bytes;
    bs->has_prev    = 0;

    bs->change_counts = (uint64_t *)calloc(block_bits, sizeof(uint64_t));
    if (!bs->change_counts) {
        return -1;
    }

    bs->prev_block = (unsigned char *)malloc(block_bytes);
    if (!bs->prev_block) {
        free(bs->change_counts);
        bs->change_counts = NULL;
        return -1;
    }

    memset(bs->prev_block, 0, block_bytes);
    return 0;
}

void bitstats_reset(BitStats *bs)
{
    if (!bs)
        return;
    if (bs->change_counts) {
        memset(bs->change_counts, 0, bs->block_bits * sizeof(uint64_t));
    }
    bs->has_prev = 0;
    if (bs->prev_block && bs->block_bytes > 0) {
        memset(bs->prev_block, 0, bs->block_bytes);
    }
}

void bitstats_free(BitStats *bs)
{
    if (!bs)
        return;
    free(bs->change_counts);
    free(bs->prev_block);
    bs->change_counts = NULL;
    bs->prev_block    = NULL;
    bs->block_bits    = 0;
    bs->block_bytes   = 0;
    bs->has_prev      = 0;
}

void bitstats_update(BitStats *bs,
                     const unsigned char *block,
                     size_t block_bytes)
{
    if (!bs || !block)
        return;
    if (block_bytes < bs->block_bytes) {
        /* block_bytes 부족한 경우는 업데이트 생략 */
        return;
    }

    size_t block_bits  = bs->block_bits;
    size_t prev_bytes  = bs->block_bytes;

    if (!bs->has_prev) {
        /* 첫 블록: 이전 값이 없으므로 변동 카운트는 증가시키지 않고
         * prev_block만 채운다.
         */
        memcpy(bs->prev_block, block, prev_bytes);
        bs->has_prev = 1;
        return;
    }

    /* bit_pos = byte_idx * 8 + bit_in_byte (LSB-first) */
    for (size_t bit_pos = 0; bit_pos < block_bits; ++bit_pos) {
        size_t byte_idx   = bit_pos / 8u;
        int    bit_in     = (int)(bit_pos % 8u);
        unsigned char mask = (unsigned char)(1u << bit_in);

        unsigned char prev_byte = bs->prev_block[byte_idx];
        unsigned char cur_byte  = block[byte_idx];

        uint8_t prev_bit = (uint8_t)((prev_byte & mask) ? 1u : 0u);
        uint8_t cur_bit  = (uint8_t)((cur_byte  & mask) ? 1u : 0u);

        if (prev_bit != cur_bit) {
            bs->change_counts[bit_pos]++;
        }
    }

    /* 현재 블록을 prev로 복사 */
    memcpy(bs->prev_block, block, prev_bytes);
}

int bitstats_accumulate(BitStats *dst, const BitStats *src)
{
    if (!dst || !src)
        return -1;
    if (dst->block_bits != src->block_bits)
        return -1;

    for (size_t i = 0; i < dst->block_bits; ++i) {
        dst->change_counts[i] += src->change_counts[i];
    }
    /* prev_block / has_prev는 그대로 둔다 (dst 기준 유지) */
    return 0;
}

void bitstats_print(const BitStats *bs,
                    const char *label,
                    FILE *out)
{
    if (!bs || !out)
        return;

    const char *name = label ? label : "(no label)";

    fprintf(out,
            "\n[BitStats] %s: block_bits=%zu, block_bytes=%zu\n",
            name, bs->block_bits, bs->block_bytes);

    for (size_t bit_pos = 0; bit_pos < bs->block_bits; ++bit_pos) {
        size_t byte_idx = bit_pos / 8u;
        int    bit_in   = (int)(bit_pos % 8u);
        uint64_t cnt    = bs->change_counts[bit_pos];

        fprintf(out,
                "  bit %3zu (byte %3zu, bit %d): %" PRIu64 "\n",
                bit_pos, byte_idx, bit_in, cnt);
    }
    fprintf(out, "\n");
}
