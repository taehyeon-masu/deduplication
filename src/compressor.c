#include "../include/compressor.h"
#include "../include/dictionary.h"
#include "../include/bin_io.h"
#include "../include/bitstats.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* ============================================================
 * Terminology
 * ------------------------------------------------------------
 *  - field : 한 block 안의 센서별 조각 (예: T(2B), RH(2B), lux1(2B), P1(4B), ...)
 *  - segment : 출력 파일을 잘라 쓰는 단위
 *      * output.ddp       : segment 0  (순수 dedup, base/dev 없음)
 *      * output.ddp.seg1  : segment 1  (base/dev 분리)
 *      * output.ddp.seg2  : segment 2, ...
 * ============================================================ */

/* ============================================================
 * Common helpers
 * ============================================================ */

static int write_u32_le(FILE *fp, uint32_t v)
{
    unsigned char b[4];
    b[0] = (unsigned char)(v & 0xFFu);
    b[1] = (unsigned char)((v >> 8) & 0xFFu);
    b[2] = (unsigned char)((v >> 16) & 0xFFu);
    b[3] = (unsigned char)((v >> 24) & 0xFFu);
    return (fwrite(b, 1, 4, fp) == 4) ? 1 : 0;
}

static int read_u32_le(FILE *fp, uint32_t *out)
{
    unsigned char b[4];
    size_t n = fread(b, 1, 4, fp);
    if (n != 4)
        return 0;
    *out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return 1;
}

/* ============================================================
 * Bit helpers
 *   - block_buf, base_buf, dev_buf 모두 LSB-first bit packing 가정
 * ============================================================ */

static inline uint8_t get_bit_from_bytes(const unsigned char *buf, size_t bit_pos)
{
    size_t byte_idx = bit_pos / 8u;
    int    bit_idx  = (int)(bit_pos % 8u); /* LSB-first */
    return (uint8_t)((buf[byte_idx] >> bit_idx) & 1u);
}

static inline void set_bit_in_bytes(unsigned char *buf, size_t bit_pos, uint8_t v)
{
    size_t byte_idx = bit_pos / 8u;
    int    bit_idx  = (int)(bit_pos % 8u);
    if (v)
        buf[byte_idx] |= (unsigned char)(1u << bit_idx);
    else
        buf[byte_idx] &= (unsigned char)~(1u << bit_idx);
}

/* ============================================================
 * Block size 계산 (multi-layout)
 *   field_sizes[]: 한 블록 안에서 각 field(센서 조각)의 바이트 수
 * ============================================================ */

static size_t compute_block_bytes_multi(int num_fields, const int *field_sizes)
{
    size_t sum = 0;
    for (int i = 0; i < num_fields; ++i)
    {
        if (field_sizes[i] <= 0)
            return 0;
        sum += (size_t)field_sizes[i];
    }
    return sum;
}

/* ============================================================
 * Deviation 길이 (비트 개수 → 바이트)
 * ============================================================ */

static size_t compute_dev_len_from_positions(int num_bits)
{
    if (num_bits <= 0)
        return 0;
    return (size_t)((num_bits + 7) / 8); /* ceil(num_bits/8) */
}

/* ============================================================
 * Dev 비트 선택 (bitstats 기반 top-n)
 * ============================================================ */

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

/* bitstats 결과에서 top-n 비트 위치를 골라 dev_positions에 채운다. */
static int select_top_n_bits_from_bitstats(const BitStats *bs,
                                           size_t top_n,
                                           int **out_positions,
                                           int *out_count)
{
    if (!bs || !out_positions || !out_count)
        return -1;

    size_t num_bits = bs->num_bits;
    if (num_bits == 0 || !bs->change_counts)
        return -1;

    if (top_n == 0 || top_n > num_bits)
        top_n = num_bits;

    BitCount *arr = (BitCount *)malloc(sizeof(BitCount) * num_bits);
    if (!arr)
        return -1;

    for (size_t i = 0; i < num_bits; ++i)
    {
        arr[i].bit_pos = i;
        arr[i].count   = bs->change_counts[i];
    }

    qsort(arr, num_bits, sizeof(BitCount), cmp_bitcount_desc);

    int *positions = (int *)malloc(sizeof(int) * top_n);
    if (!positions)
    {
        free(arr);
        return -1;
    }

    for (size_t i = 0; i < top_n; ++i)
        positions[i] = (int)arr[i].bit_pos;

    free(arr);

    *out_positions = positions;
    *out_count     = (int)top_n;
    return 0;
}

/* ============================================================
 * 전역 dev 비트 정보
 *   - bitstats 1패스로 채워짐
 * ============================================================ */

static int   *g_dev_positions     = NULL;  /* dev bit positions(원본 block 기준, 0..block_bits-1) */
static int    g_num_dev_positions = 0;

/* 가장 많이 바뀐 비트를 몇 개 dev로 사용할지 (원하면 바꿔서 사용) */
#ifndef DEV_TOP_N
#define DEV_TOP_N 64
#endif

/* bitstats를 이용해 입력 파일 전체를 1패스 스캔 → dev 비트(top-n) 자동 선택 */
static int init_dev_positions_with_bitstats(const char *input_filename,
                                            size_t block_bytes,
                                            size_t top_n)
{
    FILE *fp = fopen(input_filename, "rb");
    if (!fp)
    {
        perror("fopen for bitstats");
        return -1;
    }

    /* 파일 크기 */
    if (fseek(fp, 0, SEEK_END) != 0)
    {
        perror("fseek end (bitstats)");
        fclose(fp);
        return -1;
    }
    long file_size = ftell(fp);
    if (file_size < 0)
    {
        perror("ftell (bitstats)");
        fclose(fp);
        return -1;
    }
    if (fseek(fp, 0, SEEK_SET) != 0)
    {
        perror("fseek set (bitstats)");
        fclose(fp);
        return -1;
    }

    size_t nbytes = (size_t)file_size;
    if (nbytes < block_bytes)
    {
        fprintf(stderr,
                "bitstats: file too small for one block (block_bytes=%zu)\n",
                block_bytes);
        fclose(fp);
        return -1;
    }

    size_t num_blocks = nbytes / block_bytes;
    size_t used_bytes = num_blocks * block_bytes;
    if (used_bytes < nbytes)
    {
        fprintf(stderr,
                "bitstats: last %zu bytes ignored (not enough to fill a block)\n",
                (nbytes - used_bytes));
    }

    size_t block_bits = block_bytes * 8u;

    BitStats bs;
    if (bitstats_init(&bs, block_bits) != 0)
    {
        fprintf(stderr, "bitstats_init failed\n");
        fclose(fp);
        return -1;
    }

    unsigned char *buf = (unsigned char *)malloc(block_bytes);
    if (!buf)
    {
        fprintf(stderr, "bitstats: failed to allocate temp block\n");
        bitstats_free(&bs);
        fclose(fp);
        return -1;
    }

    for (size_t b = 0; b < num_blocks; ++b)
    {
        size_t n = fread(buf, 1, block_bytes, fp);
        if (n != block_bytes)
        {
            fprintf(stderr, "bitstats: failed to read block %zu\n", b);
            free(buf);
            bitstats_free(&bs);
            fclose(fp);
            return -1;
        }
        bitstats_update_block(&bs, buf, block_bytes);
    }

    free(buf);
    fclose(fp);

    /* 정렬된 결과 출력 (debug용) */
    fprintf(stderr, "=== Bit change counts (sorted, desc) ===\n");
    bitstats_print_sorted(stderr, &bs);

    /* top-n 비트 자동 선택 */
    if (select_top_n_bits_from_bitstats(&bs, top_n,
                                        &g_dev_positions,
                                        &g_num_dev_positions) != 0)
    {
        fprintf(stderr, "bitstats: select_top_n_bits_from_bitstats failed\n");
        bitstats_free(&bs);
        return -1;
    }

    fprintf(stderr,
            "bitstats: selected %d dev bits (top_n=%zu, block_bits=%zu)\n",
            g_num_dev_positions, top_n, block_bits);

    bitstats_free(&bs);
    return 0;
}

/* ============================================================
 * Dev bit table helpers (is_dev_bit / dev_index)
 * ============================================================ */

/* is_dev_bit[bit_pos] = 1 if dev bit, 0 otherwise */
static uint8_t *build_is_dev_bit_table(const int *dev_positions,
                                       int dev_pos_count,
                                       size_t block_bits)
{
    uint8_t *tbl = (uint8_t *)malloc(block_bits);
    if (!tbl)
        return NULL;

    memset(tbl, 0, block_bits);
    for (int i = 0; i < dev_pos_count; ++i)
    {
        int pos = dev_positions[i];
        if (pos >= 0 && (size_t)pos < block_bits)
            tbl[pos] = 1u;
    }
    return tbl;
}

/* dev_index[bit_pos] = dev bit의 인덱스 (0..dev_pos_count-1), base bit면 -1 */
static int *build_dev_index_table(const int *dev_positions,
                                  int dev_pos_count,
                                  size_t block_bits)
{
    int *tbl = (int *)malloc(sizeof(int) * block_bits);
    if (!tbl)
        return NULL;

    for (size_t i = 0; i < block_bits; ++i)
        tbl[i] = -1;

    for (int i = 0; i < dev_pos_count; ++i)
    {
        int pos = dev_positions[i];
        if (pos >= 0 && (size_t)pos < block_bits)
            tbl[pos] = i;
    }
    return tbl;
}

/* ============================================================
 * base/dev 분리 (compact base + dev bitstream)
 *
 *  - block_buf : 원본 block (block_bytes)
 *  - dev_positions : dev bit 위치 (block 내 bit offset)
 *  - is_dev_bit    : dev 여부 table (bit 단위)
 *
 *  base_buf : base_bits 비트를 bit-packed (LSB-first)
 *  dev_buf  : dev_pos_count 비트를 bit-packed (LSB-first, dev_positions 순서)
 * ============================================================ */

static void split_block_to_base_and_dev_compact(
    const unsigned char *block_buf,
    size_t block_bytes,
    const int *dev_positions,
    int dev_pos_count,
    const uint8_t *is_dev_bit,
    unsigned char *base_buf,
    size_t base_bytes,
    unsigned char *dev_buf,
    size_t dev_len_bytes)
{
    size_t block_bits = block_bytes * 8u;
    size_t base_bits  = block_bits - (size_t)dev_pos_count;

    (void)base_bytes;
    (void)dev_len_bytes;

    /* 초기화 */
    memset(base_buf, 0, base_bytes);
    memset(dev_buf,  0, dev_len_bytes);

    /* base bits: dev가 아닌 bit들을 0..base_bits-1 순서로 채운다 */
    size_t base_bit_idx = 0;
    for (size_t bit_pos = 0; bit_pos < block_bits; ++bit_pos)
    {
        if (is_dev_bit[bit_pos])
            continue;

        uint8_t bit = get_bit_from_bytes(block_buf, bit_pos);
        set_bit_in_bytes(base_buf, base_bit_idx, bit);
        base_bit_idx++;
    }

    /* dev bits: dev_positions[] 순서대로 dev_buf에 채운다 */
    for (int i = 0; i < dev_pos_count; ++i)
    {
        int pos = dev_positions[i];
        uint8_t bit = get_bit_from_bytes(block_buf, (size_t)pos);
        set_bit_in_bytes(dev_buf, (size_t)i, bit);
    }

    /* sanity check (디버그용, 필요하면 주석 해제) */
    /*
    if (base_bit_idx != base_bits) {
        fprintf(stderr, "split_block_to_base_and_dev_compact: base_bit_idx=%zu, expected=%zu\n",
                base_bit_idx, base_bits);
    }
    */
}

/* ============================================================
 * base/dev 합치기 (compact base + dev bitstream → 원본 block)
 *
 *  - base_buf : base_bits 비트(bit-packed)
 *  - dev_buf  : dev_pos_count 비트(bit-packed)
 *  - is_dev_bit, dev_index : dev 여부 및 dev index 테이블
 * ============================================================ */

static void merge_base_and_dev_compact(
    const unsigned char *base_buf,
    size_t base_bytes,
    const unsigned char *dev_buf,
    size_t dev_len_bytes,
    size_t block_bytes,
    const uint8_t *is_dev_bit,
    const int *dev_index,
    int dev_pos_count,
    unsigned char *out_block)
{
    size_t block_bits = block_bytes * 8u;
    size_t base_bits  = block_bits - (size_t)dev_pos_count;

    (void)base_bytes;
    (void)dev_len_bytes;

    memset(out_block, 0, block_bytes);

    size_t base_bit_idx = 0;

    for (size_t bit_pos = 0; bit_pos < block_bits; ++bit_pos)
    {
        uint8_t bit;
        if (is_dev_bit[bit_pos])
        {
            int idx = dev_index[bit_pos];
            if (idx < 0)
                bit = 0;
            else
                bit = get_bit_from_bytes(dev_buf, (size_t)idx);
        }
        else
        {
            if (base_bit_idx >= base_bits)
                bit = 0;
            else
                bit = get_bit_from_bytes(base_buf, base_bit_idx);
            base_bit_idx++;
        }
        set_bit_in_bytes(out_block, bit_pos, bit);
    }
}

/* ============================================================
 * 하나의 segment를 DDP1 파일로 쓰는 helper
 *
 *  - block_bytes : 원본 block 크기 (field_sizes 합)
 *  - base_bytes  : dictionary에 저장되는 base block 크기
 *  - dev_pos_count : deviation bit 개수 (#bits)
 *  - dev_len_per_block : deviation bitstream 길이 (bytes)
 *
 *  segment 0  : dev_pos_count=0, dev_len_per_block=0, base_bytes=block_bytes
 *  segment>=1 : dev_pos_count>0, base_bytes < block_bytes
 * ============================================================ */

static int write_ddp1_segment(const char *base_out,
                              int segment_idx,
                              size_t block_bytes,
                              size_t base_bytes,
                              int num_fields,
                              const int *field_sizes,
                              int dev_pos_count,
                              const int *dev_positions,
                              size_t dev_len_per_block,
                              const Dictionary *dict,
                              const uint8_t *block_ids,
                              const unsigned char *dev_stream,
                              size_t num_blocks_segment)
{
    if (num_blocks_segment == 0)
        return 0;

    char filename[1024];
    if (segment_idx == 0)
        snprintf(filename, sizeof(filename), "%s", base_out);
    else
        snprintf(filename, sizeof(filename), "%s.seg%d", base_out, segment_idx);

    FILE *fp = fopen(filename, "wb");
    if (!fp)
    {
        perror("fopen output segment");
        return 1;
    }

    const unsigned char magic[4] = {'D', 'D', 'P', '1'};
    if (fwrite(magic, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to write magic (segment)\n");
        fclose(fp);
        return 1;
    }

    uint32_t block_bytes_u32   = (uint32_t)block_bytes;
    uint32_t num_fields_u32    = (uint32_t)num_fields;
    uint32_t dict_size_u32     = (uint32_t)dict->size;
    uint32_t num_blocks_u32    = (uint32_t)num_blocks_segment;
    uint32_t dev_pos_count_u32 = (uint32_t)dev_pos_count;      /* #bits */
    uint32_t dev_len_u32       = (uint32_t)dev_len_per_block;  /* bytes */

    if (!write_u32_le(fp, block_bytes_u32) ||
        !write_u32_le(fp, num_fields_u32)  ||
        !write_u32_le(fp, dict_size_u32)   ||
        !write_u32_le(fp, num_blocks_u32)  ||
        !write_u32_le(fp, dev_pos_count_u32) ||
        !write_u32_le(fp, dev_len_u32))
    {
        fprintf(stderr, "Failed to write DDP1 segment header\n");
        fclose(fp);
        return 1;
    }

    /* field_sizes[] */
    for (int f = 0; f < num_fields; ++f)
    {
        uint32_t v = (uint32_t)field_sizes[f];
        if (!write_u32_le(fp, v))
        {
            fprintf(stderr, "Failed to write field_sizes[%d] (segment)\n", f);
            fclose(fp);
            return 1;
        }
    }

    /* dev_positions[] (bit offsets) */
    for (int i = 0; i < dev_pos_count; ++i)
    {
        uint32_t v = (uint32_t)dev_positions[i];
        if (!write_u32_le(fp, v))
        {
            fprintf(stderr, "Failed to write dev_positions[%d] (segment)\n", i);
            fclose(fp);
            return 1;
        }
    }

    /* dictionary blocks (base_bytes 길이) */
    for (int i = 0; i < dict->size; ++i)
    {
        if (fwrite(dict->blocks[i], 1, base_bytes, fp) != base_bytes)
        {
            fprintf(stderr, "Failed to write dictionary block %d (segment)\n", i);
            fclose(fp);
            return 1;
        }
    }

    /* block_ids (1 byte씩) */
    for (size_t b = 0; b < num_blocks_segment; ++b)
    {
        uint8_t id = block_ids[b];
        if (fwrite(&id, 1, 1, fp) != 1)
        {
            fprintf(stderr, "Failed to write 1-byte block id %zu (segment)\n", b);
            fclose(fp);
            return 1;
        }
    }

    /* deviation stream (bit-packed) : segment 0에서는 dev_len_per_block=0 → skip */
    if (dev_len_per_block > 0 && dev_stream)
    {
        size_t dev_total_bytes = num_blocks_segment * dev_len_per_block;
        if (fwrite(dev_stream, 1, dev_total_bytes, fp) != dev_total_bytes)
        {
            fprintf(stderr, "Failed to write deviation stream (segment)\n");
            fclose(fp);
            return 1;
        }
    }

    fclose(fp);

    fprintf(stderr,
            "[segment %d] wrote '%s': blocks=%zu, dict_size=%d, "
            "block_bytes=%zu, base_bytes=%zu, dev_len_per_block=%zu (bytes), "
            "dev_pos_count=%d (bits)\n",
            segment_idx, filename, num_blocks_segment, dict->size,
            block_bytes, base_bytes, dev_len_per_block, dev_pos_count);

    return 0;
}

/* ============================================================
 * Compression (multi-layout, segmented)
 *   - segment 0 : 순수 dedup (dev 없음, dictionary block = full block)
 *   - segment>=1: base/dev 분리, base compact + dev bitstream
 * ============================================================ */

int compress_file(const char *input_filename,
                  const char *output_filename,
                  int num_fields,
                  const int *field_sizes)
{
    if (num_fields <= 0 || !field_sizes)
    {
        fprintf(stderr, "compress_file_multi: invalid num_fields\n");
        return 1;
    }

    size_t block_bytes = compute_block_bytes_multi(num_fields, field_sizes);
    if (block_bytes == 0)
    {
        fprintf(stderr, "compress_file_multi: invalid field_sizes (sum == 0 or negative)\n");
        return 1;
    }

    /* 1단계: bitstats로 dev 비트(top-n) 결정 (전역 g_dev_positions 사용) */
    if (!g_dev_positions || g_num_dev_positions <= 0)
    {
        if (init_dev_positions_with_bitstats(input_filename,
                                             block_bytes,
                                             DEV_TOP_N) != 0)
        {
            fprintf(stderr, "compress_file_multi: failed to init dev positions via bitstats\n");
            return 1;
        }
    }

    size_t block_bits = block_bytes * 8u;

    int dev_pos_count_total = g_num_dev_positions;
    if (dev_pos_count_total < 0)
        dev_pos_count_total = 0;
    if ((size_t)dev_pos_count_total > block_bits)
    {
        fprintf(stderr,
                "compress_file_multi: dev_pos_count(%d) > block_bits(%zu)\n",
                dev_pos_count_total, block_bits);
        return 1;
    }

    size_t dev_len_per_block = compute_dev_len_from_positions(dev_pos_count_total);
    size_t base_bits_total   = block_bits - (size_t)dev_pos_count_total;
    size_t base_bytes_total  = (size_t)((base_bits_total + 7) / 8u);

    /* dev_positions 범위 검증 */
    for (int i = 0; i < dev_pos_count_total; ++i)
    {
        int bit_pos = g_dev_positions[i];
        if (bit_pos < 0 || (size_t)bit_pos >= block_bits)
        {
            fprintf(stderr,
                    "compress_file_multi: dev bit position %d out of range (block_bits=%zu)\n",
                    bit_pos, block_bits);
            return 1;
        }
    }

    /* dev 여부 table (segment>=1에서 base/dev 분리에 사용) */
    uint8_t *is_dev_bit = NULL;
    if (dev_pos_count_total > 0)
    {
        is_dev_bit = build_is_dev_bit_table(g_dev_positions,
                                            dev_pos_count_total,
                                            block_bits);
        if (!is_dev_bit)
        {
            fprintf(stderr, "compress_file_multi: build_is_dev_bit_table failed\n");
            return 1;
        }
    }

    /* 입력 파일 열기 (compression pass) */
    FILE *fin = fopen(input_filename, "rb");
    if (!fin)
    {
        perror("fopen input");
        free(is_dev_bit);
        return 1;
    }

    if (fseek(fin, 0, SEEK_END) != 0)
    {
        perror("fseek end");
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }
    long file_size = ftell(fin);
    if (file_size < 0)
    {
        perror("ftell");
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }
    if (fseek(fin, 0, SEEK_SET) != 0)
    {
        perror("fseek set");
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }

    size_t nbytes = (size_t)file_size;
    if (nbytes < block_bytes)
    {
        fprintf(stderr, "Input file too small for one multi-layout block\n");
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }

    size_t num_blocks_total = nbytes / block_bytes;
    if (num_blocks_total == 0)
    {
        fprintf(stderr, "No full blocks found (multi-layout)\n");
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }

    size_t used_bytes = num_blocks_total * block_bytes;
    if (used_bytes < nbytes)
    {
        fprintf(stderr,
                "Warning: last %zu bytes are ignored (not enough to fill a block)\n",
                (nbytes - used_bytes));
    }

    /* 전체 block에 대한 block_ids / dev_stream 버퍼 */
    uint8_t *block_ids = (uint8_t *)malloc(sizeof(uint8_t) * num_blocks_total);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }

    unsigned char *dev_stream = NULL;
    if (dev_len_per_block > 0)
    {
        size_t dev_total_bytes = num_blocks_total * dev_len_per_block;
        dev_stream = (unsigned char *)malloc(dev_total_bytes);
        if (!dev_stream)
        {
            fprintf(stderr, "Failed to allocate dev_stream\n");
            free(block_ids);
            fclose(fin);
            free(is_dev_bit);
            return 1;
        }
    }

    /* dictionary (segment별로 block size 다름)
     *   - segment 0 : block_bytes (full block)
     *   - segment>=1: base_bytes_total
     */
    Dictionary dict;
    dict_init(&dict, block_bytes); /* segment 0용 */

    unsigned char *block_buf = (unsigned char *)malloc(block_bytes);
    unsigned char *base_buf  = (base_bytes_total > 0)
                               ? (unsigned char *)malloc(base_bytes_total)
                               : NULL;
    unsigned char *dev_buf   = (dev_len_per_block > 0)
                               ? (unsigned char *)malloc(dev_len_per_block)
                               : NULL;

    if (!block_buf || (base_bytes_total > 0 && !base_buf) ||
        (dev_len_per_block > 0 && !dev_buf))
    {
        fprintf(stderr, "Failed to allocate block/base/dev buffer\n");
        free(block_buf);
        free(base_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        free(is_dev_bit);
        return 1;
    }

    int    segment_idx         = 0; /* 0 -> 순수 dedup, 1+ -> base/dev */
    size_t global_b            = 0; /* 전체 block index */
    size_t segment_start_block = 0; /* 이 segment가 시작되는 global block index */
    size_t segment_block_count = 0; /* 이 segment 안에 들어가는 block 수 */

    while (global_b < num_blocks_total)
    {
        /* 원본 block 읽기 */
        size_t n = fread(block_buf, 1, block_bytes, fin);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read block %zu (multi)\n", global_b);
            free(block_buf);
            free(base_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            free(is_dev_bit);
            return 1;
        }

        /* 이 block을 어느 segment/dict에 넣을지 결정하기 위해,
         * 필요하다면 segment flush를 먼저 처리해야 한다.
         * flush는 "새로운 unique base를 넣으려는데 dict.size == 255"인 순간 수행.
         * flush 후에는 같은 block을 새로운 segment 설정으로 다시 처리한다.
         */

        while (1)
        {
            unsigned char *key_ptr = NULL;

            if (segment_idx == 0)
            {
                /* segment 0: 순수 dedup → full block을 key로 사용 (base/dev 없음) */
                key_ptr = block_buf;
            }
            else
            {
                /* segment>=1: base/dev 분리 */
                if (dev_pos_count_total > 0)
                {
                    split_block_to_base_and_dev_compact(
                        block_buf,
                        block_bytes,
                        g_dev_positions,
                        dev_pos_count_total,
                        is_dev_bit,
                        base_buf,
                        base_bytes_total,
                        dev_buf,
                        dev_len_per_block);

                    memcpy(dev_stream + global_b * dev_len_per_block,
                           dev_buf,
                           dev_len_per_block);
                }
                key_ptr = base_buf;
            }

            int idx = dict_find(&dict, key_ptr);

            if (idx == -1 && dict.size >= 255)
            {
                /* ---- 현재 segment flush ---- */
                size_t blocks_in_segment = segment_block_count;
                if (blocks_in_segment > 0)
                {
                    const uint8_t *segment_block_ids =
                        block_ids + segment_start_block;
                    const unsigned char *segment_dev_stream = NULL;

                    int    seg_dev_pos_count = 0;
                    size_t seg_dev_len       = 0;
                    size_t seg_base_bytes    = block_bytes;     /* default */

                    if (segment_idx == 0)
                    {
                        /* segment 0: dev 없음, base = full block */
                        seg_dev_pos_count = 0;
                        seg_dev_len       = 0;
                        seg_base_bytes    = block_bytes;
                    }
                    else
                    {
                        /* segment>=1: dev 사용, base compact */
                        seg_dev_pos_count = dev_pos_count_total;
                        seg_dev_len       = dev_len_per_block;
                        seg_base_bytes    = base_bytes_total;

                        if (dev_stream && dev_len_per_block > 0)
                        {
                            segment_dev_stream =
                                dev_stream + segment_start_block * dev_len_per_block;
                        }
                    }

                    if (write_ddp1_segment(output_filename,
                                           segment_idx,
                                           block_bytes,
                                           seg_base_bytes,
                                           num_fields,
                                           field_sizes,
                                           seg_dev_pos_count,
                                           (seg_dev_pos_count > 0) ? g_dev_positions : NULL,
                                           seg_dev_len,
                                           &dict,
                                           segment_block_ids,
                                           segment_dev_stream,
                                           blocks_in_segment) != 0)
                    {
                        fprintf(stderr, "Failed to write segment %d\n",
                                segment_idx);
                        free(block_buf);
                        free(base_buf);
                        free(dev_buf);
                        free(dev_stream);
                        free(block_ids);
                        dict_free(&dict);
                        fclose(fin);
                        free(is_dev_bit);
                        return 1;
                    }
                }

                /* ---- 새 segment 시작 ---- */
                dict_free(&dict);
                segment_idx++;

                if (segment_idx == 0)
                {
                    /* 이론상 오지 않음 */
                    dict_init(&dict, block_bytes);
                }
                else
                {
                    /* segment>=1 : base_bytes_total 크기의 base dictionary */
                    dict_init(&dict, base_bytes_total);
                }

                segment_start_block = global_b;
                segment_block_count = 0;

                /* 동일 block을 새 segment 설정으로 다시 처리 */
                continue;
            }

            /* 여기까지 왔으면 flush 없이 진행 가능 */
            if (idx == -1)
            {
                idx = dict_add(&dict, key_ptr);
                if (idx < 0)
                {
                    fprintf(stderr, "dict_add failed\n");
                    free(block_buf);
                    free(base_buf);
                    free(dev_buf);
                    free(dev_stream);
                    free(block_ids);
                    dict_free(&dict);
                    fclose(fin);
                    free(is_dev_bit);
                    return 1;
                }
            }

            if (idx < 0 || idx > 255)
            {
                fprintf(stderr,
                        "Dictionary index out of range (idx=%d). "
                        "Expect 0..255 for 1-byte IDs.\n",
                        idx);
                free(block_buf);
                free(base_buf);
                free(dev_buf);
                free(dev_stream);
                free(block_ids);
                dict_free(&dict);
                fclose(fin);
                free(is_dev_bit);
                return 1;
            }

            block_ids[global_b] = (uint8_t)idx;
            segment_block_count++;
            break; /* while(1) 탈출 → 다음 block */
        }

        global_b++;
    }

    fclose(fin);

    /* 마지막 segment flush */
    if (segment_block_count > 0)
    {
        const uint8_t *segment_block_ids =
            block_ids + segment_start_block;
        const unsigned char *segment_dev_stream = NULL;

        int    seg_dev_pos_count = 0;
        size_t seg_dev_len       = 0;
        size_t seg_base_bytes    = block_bytes;

        if (segment_idx == 0)
        {
            /* segment 0 : dev 없음 */
            seg_dev_pos_count = 0;
            seg_dev_len       = 0;
            seg_base_bytes    = block_bytes;
        }
        else
        {
            seg_dev_pos_count = dev_pos_count_total;
            seg_dev_len       = dev_len_per_block;
            seg_base_bytes    = base_bytes_total;

            if (dev_stream && dev_len_per_block > 0)
            {
                segment_dev_stream =
                    dev_stream + segment_start_block * dev_len_per_block;
            }
        }

        if (write_ddp1_segment(output_filename,
                               segment_idx,
                               block_bytes,
                               seg_base_bytes,
                               num_fields,
                               field_sizes,
                               seg_dev_pos_count,
                               (seg_dev_pos_count > 0) ? g_dev_positions : NULL,
                               seg_dev_len,
                               &dict,
                               segment_block_ids,
                               segment_dev_stream,
                               segment_block_count) != 0)
        {
            fprintf(stderr, "Failed to write final segment %d\n", segment_idx);
            free(block_buf);
            free(base_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            free(is_dev_bit);
            return 1;
        }
    }

    free(block_buf);
    free(base_buf);
    free(dev_buf);
    free(dev_stream);
    free(block_ids);
    dict_free(&dict);
    free(is_dev_bit);

    /* 전역 dev_positions는 여기서 free할 수도 있고, 재사용할 수도 있음.
       일단 한 번만 쓰는 용도라고 보고 free. */
    free(g_dev_positions);
    g_dev_positions     = NULL;
    g_num_dev_positions = 0;

    fprintf(stderr,
            "Compressed (segmented, segment0=pure dedup, segment>=1=base/dev): "
            "used_bytes=%zu, block_bytes=%zu, total_blocks=%zu\n",
            used_bytes, block_bytes, num_blocks_total);

    return 0;
}

/* ============================================================
 * Decompression
 *   - 각 segment 파일을 개별적으로 복원
 *   - segment 0 : dev_pos_count=0 → 순수 dedup
 *   - segment>=1: dev_pos_count>0 → base/dev 합쳐 복원
 * ============================================================ */

int decompress_file(const char *input_filename,
                    const char *output_filename)
{
    FILE *fp = fopen(input_filename, "rb");
    if (!fp)
    {
        perror("fopen compressed");
        return 1;
    }

    unsigned char magic[4];
    if (fread(magic, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to read magic (multi dec)\n");
        fclose(fp);
        return 1;
    }
    if (!(magic[0] == 'D' && magic[1] == 'D' &&
          magic[2] == 'P' && magic[3] == '1'))
    {
        fprintf(stderr, "Invalid magic for DDP1 in decompress_file\n");
        fclose(fp);
        return 1;
    }

    uint32_t block_bytes_u32;
    uint32_t num_fields_u32;
    uint32_t dict_size_u32;
    uint32_t num_blocks_u32;
    uint32_t dev_pos_count_u32;
    uint32_t dev_len_u32;

    if (!read_u32_le(fp, &block_bytes_u32) ||
        !read_u32_le(fp, &num_fields_u32)  ||
        !read_u32_le(fp, &dict_size_u32)   ||
        !read_u32_le(fp, &num_blocks_u32)  ||
        !read_u32_le(fp, &dev_pos_count_u32) ||
        !read_u32_le(fp, &dev_len_u32))
    {
        fprintf(stderr,
                "Failed to read DDP1 header (block_bytes/num_fields/"
                "dict_size/num_blocks/dev_pos_count/dev_len)\n");
        fclose(fp);
        return 1;
    }

    size_t block_bytes       = (size_t)block_bytes_u32;
    int    num_fields        = (int)num_fields_u32;
    size_t dict_size         = (size_t)dict_size_u32;
    size_t num_blocks        = (size_t)num_blocks_u32;
    int    dev_pos_count     = (int)dev_pos_count_u32; /* #bits */
    size_t dev_len_per_block = (size_t)dev_len_u32;    /* bytes */

    if (num_fields <= 0)
    {
        fprintf(stderr, "Invalid num_fields in DDP1 header\n");
        fclose(fp);
        return 1;
    }
    if (dev_pos_count < 0)
    {
        fprintf(stderr, "Invalid dev_pos_count in DDP1 header\n");
        fclose(fp);
        return 1;
    }
    if (dict_size > 255)
    {
        fprintf(stderr,
                "DDP1 file dict_size=%zu > 255, but decoder expects 1-byte block ids.\n",
                dict_size);
        fclose(fp);
        return 1;
    }

    size_t block_bits   = block_bytes * 8u;
    size_t base_bits    = block_bits - (size_t)dev_pos_count;
    size_t base_bytes   = (size_t)((base_bits + 7) / 8u); /* segment 0이면 block_bytes와 같음 */

    /* field_sizes 읽기 */
    int *field_sizes = (int *)malloc(sizeof(int) * num_fields);
    if (!field_sizes)
    {
        fprintf(stderr, "Failed to allocate field_sizes (dec)\n");
        fclose(fp);
        return 1;
    }

    size_t sum_bytes = 0;
    for (int f = 0; f < num_fields; ++f)
    {
        uint32_t v;
        if (!read_u32_le(fp, &v))
        {
            fprintf(stderr, "Failed to read field_sizes[%d] in DDP1\n", f);
            free(field_sizes);
            fclose(fp);
            return 1;
        }
        field_sizes[f] = (int)v;
        sum_bytes += (size_t)v;
    }

    if (sum_bytes != block_bytes)
    {
        fprintf(stderr,
                "Warning: sum(field_sizes)=%zu != block_bytes=%zu in header\n",
                sum_bytes, block_bytes);
    }

    /* deviation positions 읽기 (bit offsets) */
    int *dev_positions = NULL;
    if (dev_pos_count > 0)
    {
        dev_positions = (int *)malloc(sizeof(int) * dev_pos_count);
        if (!dev_positions)
        {
            fprintf(stderr, "Failed to allocate dev_positions (dec)\n");
            free(field_sizes);
            fclose(fp);
            return 1;
        }

        for (int i = 0; i < dev_pos_count; ++i)
        {
            uint32_t v;
            if (!read_u32_le(fp, &v))
            {
                fprintf(stderr, "Failed to read dev_positions[%d] in DDP1\n", i);
                free(dev_positions);
                free(field_sizes);
                fclose(fp);
                return 1;
            }
            dev_positions[i] = (int)v;
            if (dev_positions[i] < 0 || (size_t)dev_positions[i] >= block_bits)
            {
                fprintf(stderr,
                        "dev_positions[%d]=%d out of range (block_bits=%zu)\n",
                        i, dev_positions[i], block_bits);
                free(dev_positions);
                free(field_sizes);
                fclose(fp);
                return 1;
            }
        }

        /* dev_len_per_block 검증 (선택사항) */
        size_t expected_dev_len = compute_dev_len_from_positions(dev_pos_count);
        if (expected_dev_len != dev_len_per_block)
        {
            fprintf(stderr,
                    "Warning: dev_len_per_block in header (%zu) != computed (%zu)\n",
                    dev_len_per_block, expected_dev_len);
        }
    }

    /* dev 여부 / index 테이블 (segment>=1에서만 사용) */
    uint8_t *is_dev_bit = NULL;
    int     *dev_index  = NULL;
    if (dev_pos_count > 0)
    {
        is_dev_bit = build_is_dev_bit_table(dev_positions,
                                            dev_pos_count,
                                            block_bits);
        dev_index  = build_dev_index_table(dev_positions,
                                           dev_pos_count,
                                           block_bits);
        if (!is_dev_bit || !dev_index)
        {
            fprintf(stderr, "Failed to build dev tables (dec)\n");
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            fclose(fp);
            return 1;
        }
    }

    /* dictionary 읽기 (base_bytes 크기) */
    Dictionary dict;
    dict_init(&dict, base_bytes);

    for (size_t i = 0; i < dict_size; ++i)
    {
        unsigned char *buf = (unsigned char *)malloc(base_bytes);
        if (!buf)
        {
            fprintf(stderr, "Failed to allocate block buffer (dict read, multi)\n");
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        size_t n = fread(buf, 1, base_bytes, fp);
        if (n != base_bytes)
        {
            fprintf(stderr, "Failed to read dictionary block %zu (multi)\n", i);
            free(buf);
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        dict_add(&dict, buf);
        free(buf);
    }

    /* block_ids 읽기 (1 byte씩) */
    uint8_t *block_ids = (uint8_t *)malloc(sizeof(uint8_t) * num_blocks);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids (multi dec)\n");
        free(is_dev_bit);
        free(dev_index);
        free(dev_positions);
        free(field_sizes);
        dict_free(&dict);
        fclose(fp);
        return 1;
    }

    for (size_t b = 0; b < num_blocks; ++b)
    {
        int c = fgetc(fp);
        if (c == EOF)
        {
            fprintf(stderr, "Failed to read 1-byte block id %zu (multi dec)\n", b);
            free(block_ids);
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        block_ids[b] = (uint8_t)c;
    }

    /* deviation stream 읽기 (segment 0이면 dev_len_per_block=0 → skip) */
    size_t dev_total_bytes = num_blocks * dev_len_per_block;
    unsigned char *dev_stream = NULL;
    if (dev_total_bytes > 0)
    {
        dev_stream = (unsigned char *)malloc(dev_total_bytes);
        if (!dev_stream)
        {
            fprintf(stderr, "Failed to allocate dev_stream (multi dec)\n");
            free(block_ids);
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        size_t n = fread(dev_stream, 1, dev_total_bytes, fp);
        if (n != dev_total_bytes)
        {
            fprintf(stderr, "Failed to read deviation stream (multi dec)\n");
            free(dev_stream);
            free(block_ids);
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
    }

    fclose(fp);

    /* 복원 버퍼 */
    size_t total_bytes = num_blocks * block_bytes;
    unsigned char *out = (unsigned char *)malloc(total_bytes);
    unsigned char *tmp = (unsigned char *)malloc(block_bytes);
    if (!out || !tmp)
    {
        fprintf(stderr, "Failed to allocate output buffer (multi dec)\n");
        free(out);
        free(tmp);
        free(dev_stream);
        free(block_ids);
        free(is_dev_bit);
        free(dev_index);
        free(dev_positions);
        free(field_sizes);
        dict_free(&dict);
        return 1;
    }

    for (size_t b = 0; b < num_blocks; ++b)
    {
        uint32_t id = block_ids[b];
        if (id >= (uint32_t)dict.size)
        {
            fprintf(stderr, "Invalid dictionary id %u at block %zu (multi dec)\n",
                    id, b);
            free(out);
            free(tmp);
            free(dev_stream);
            free(block_ids);
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            return 1;
        }

        const unsigned char *base_block = dict.blocks[id];

        if (dev_pos_count == 0)
        {
            /* segment 0: dev 없음 → base_block은 full block */
            memcpy(tmp, base_block, block_bytes);
        }
        else
        {
            const unsigned char *dev_ptr =
                (dev_stream && dev_len_per_block > 0)
                ? (dev_stream + b * dev_len_per_block)
                : NULL;

            if (!dev_ptr)
            {
                fprintf(stderr, "Missing dev_ptr for block %zu (multi dec)\n", b);
                free(out);
                free(tmp);
                free(dev_stream);
                free(block_ids);
                free(is_dev_bit);
                free(dev_index);
                free(dev_positions);
                free(field_sizes);
                dict_free(&dict);
                return 1;
            }

            merge_base_and_dev_compact(
                base_block,
                base_bytes,
                dev_ptr,
                dev_len_per_block,
                block_bytes,
                is_dev_bit,
                dev_index,
                dev_pos_count,
                tmp);
        }

        size_t offset = b * block_bytes;
        if (offset + block_bytes > total_bytes)
        {
            fprintf(stderr, "Output buffer overflow risk (multi dec)\n");
            free(out);
            free(tmp);
            free(dev_stream);
            free(block_ids);
            free(is_dev_bit);
            free(dev_index);
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            return 1;
        }

        memcpy(out + offset, tmp, block_bytes);
    }

    int ret = write_binary_file(output_filename, out, total_bytes);

    free(out);
    free(tmp);
    free(dev_stream);
    free(block_ids);
    free(is_dev_bit);
    free(dev_index);
    free(dev_positions);
    free(field_sizes);
    dict_free(&dict);

    return ret;
}
