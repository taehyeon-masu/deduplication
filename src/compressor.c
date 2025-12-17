#include "../include/compressor.h"
#include "../include/dictionary.h"
#include "../include/bin_io.h"
#include "../include/bitstats.h"   /* <-- 비트 통계용 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* ============================================================
 * Terminology
 * ------------------------------------------------------------
 *  - field:  한 block 안의 센서별 조각 (예: T(2B), RH(2B), lux1(2B), P1(4B), ...)
 *  - segment: 출력 파일을 잘라 쓰는 단위
 *      * output.ddp       : segment 0
 *      * output.ddp.seg1  : segment 1
 *      * output.ddp.seg2  : segment 2
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
 * Bit helpers (block / deviation bitstream)
 * ============================================================ */

/* block(bit_pos)에서 비트 하나 읽기: LSB-first */
static inline uint8_t get_block_bit(const unsigned char *buf, size_t bit_pos)
{
    size_t byte_idx = bit_pos / 8;
    int    bit_idx  = (int)(bit_pos % 8); /* LSB-first */
    return (uint8_t)((buf[byte_idx] >> bit_idx) & 1u);
}

/* block(bit_pos)에 비트 하나 쓰기: LSB-first */
static inline void set_block_bit(unsigned char *buf, size_t bit_pos, uint8_t v)
{
    size_t byte_idx = bit_pos / 8;
    int    bit_idx  = (int)(bit_pos % 8);
    if (v)
        buf[byte_idx] |= (unsigned char)(1u << bit_idx);
    else
        buf[byte_idx] &= (unsigned char)~(1u << bit_idx);
}

/* deviation bitstream에서 i번째 비트 읽기 (LSB-first) */
static inline uint8_t get_dev_bit(const unsigned char *buf, size_t bit_idx)
{
    size_t byte_idx = bit_idx / 8;
    int    bit_in   = (int)(bit_idx % 8);
    return (uint8_t)((buf[byte_idx] >> bit_in) & 1u);
}

/* deviation bitstream에 i번째 비트 쓰기 (LSB-first) */
static inline void set_dev_bit(unsigned char *buf, size_t bit_idx, uint8_t v)
{
    size_t byte_idx = bit_idx / 8;
    int    bit_in   = (int)(bit_idx % 8);
    if (v)
        buf[byte_idx] |= (unsigned char)(1u << bit_in);
    else
        buf[byte_idx] &= (unsigned char)~(1u << bit_in);
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
 * Deviation: bit-position 기반 설정
 *
 * - 압축 시:
 *   dev_positions[] 에 적힌 "block 내 비트 오프셋"에서만 비트를 뽑아
 *   dev_buf(bitstream)에 packing.
 *
 * - dev_pos_count   : deviation bit 개수 (#bits)
 * - dev_len_per_block: deviation bitstream의 길이 (바이트)
 *      dev_len_per_block = ceil(dev_pos_count / 8)
 *
 *  예) dev_positions = { 0, 1, 2 }  → dev_pos_count=3 → dev_len_per_block=1
 *      dev_positions = { 0..7 }     → dev_pos_count=8 → dev_len_per_block=1
 *      dev_positions = { 0..15 }    → dev_pos_count=16 → dev_len_per_block=2
 *
 *  디코딩 시: 헤더에 저장된 dev_positions[]를 그대로 사용
 *
 *  비트 packing 규칙:
 *    - i번째 deviation bit (0-based)는
 *        dev_buf[ i/8 ] 의 (i%8)-th bit (LSB 기준)에 저장
 * ============================================================ */

/*
 * (압축기에서 기본 deviation 비트 위치 설정 예시)
 *
 *  - g_dev_positions[]: block 내 비트 오프셋 (0 ~ block_bytes*8-1)
 *
 *  필요에 따라 bit 위치를 자유롭게 바꿔서 사용할 수 있음.
 *
 *  아래 예시는 그냥 예시용 패턴이니 실제로는 실험하면서 조정하면 됨.
 */
static const int g_dev_positions[] = {
    0,1,2,                        /* byte 0, bits 0~2 */
    16,17,18,19,                  /* byte 2, bits 0~3 */
    32,33,34,35,36,37,38,39,40,41,42, /* ... 예시 */
    48,49,50,51,52,53,
    64,65,
    80,81,
    96,97,98,99,100,101,102,103,
    112,113,114,115,116,117,118,119,
    128,129,130,131,132,133,134,135
};

static const int g_num_dev_positions =
    (int)(sizeof(g_dev_positions) / sizeof(g_dev_positions[0]));

/* num_bits(=dev_pos_count)를 dev_len_per_block(바이트)로 변환 */
static size_t compute_dev_len_from_positions(int num_bits)
{
    if (num_bits <= 0)
        return 0;
    return (size_t)((num_bits + 7) / 8); /* ceil(num_bits/8) */
}

/* block_buf에서 선택된 비트를 deviation bitstream(dev_buf)에 모으고,
 * block_buf의 해당 비트는 0으로 만들어 base만 남긴다.
 *
 *  - dev_positions : block 내 비트 오프셋들 (0 ~ block_bytes*8-1)
 *  - num_dev_bits  : deviation bit 개수 (#bits)
 *  - dev_buf       : 최소 dev_len_per_block 만큼 할당
 *      * dev_buf는 bitstream (LSB-first) 으로 사용
 *
 * dev_buf의 나머지 padding bit들은 0으로 채움.
 *
 * 반환값: dev_len_per_block (정상일 때)
 */
static size_t extract_base_and_deviation_by_pos(
    unsigned char *block_buf,
    size_t block_bytes,
    const int *dev_positions,
    int num_dev_bits,
    unsigned char *dev_buf,
    size_t dev_len_per_block)
{
    if (!block_buf || !dev_positions || !dev_buf || num_dev_bits < 0)
        return 0;

    size_t block_bits = block_bytes * 8;

    /* dev_buf 전체를 0으로 초기화 (padding 포함) */
    for (size_t i = 0; i < dev_len_per_block; ++i)
        dev_buf[i] = 0;

    /* deviation bitstream 채우기 */
    for (int i = 0; i < num_dev_bits; ++i)
    {
        int bit_pos = dev_positions[i];
        if (bit_pos < 0 || (size_t)bit_pos >= block_bits)
        {
            fprintf(stderr,
                    "extract_base_and_deviation_by_pos: invalid bit_pos=%d (block_bits=%zu)\n",
                    bit_pos, block_bits);
            return 0;
        }

        uint8_t bit = get_block_bit(block_buf, (size_t)bit_pos);
        set_dev_bit(dev_buf, (size_t)i, bit);

        /* base에서는 해당 비트를 0으로 클리어 */
        set_block_bit(block_buf, (size_t)bit_pos, 0);
    }

    return dev_len_per_block;
}

/* base_block + dev_buf(bitstream) → out_block (bit 위치 기반 복원)
 *
 *  - base_block: dict에 저장된 base 블록 (dev bit 위치는 0이어야 함)
 *  - dev_buf   : dev_len_per_block 바이트짜리 deviation bitstream
 *  - dev_positions: deviation bit들이 들어갈 block bit 오프셋들
 *
 * 반환값: 실제 deviation bit 개수(raw) — 보통 num_dev_bits 와 같음
 */
static size_t merge_base_and_deviation_by_pos(
    const unsigned char *base_block,
    size_t block_bytes,
    const int *dev_positions,
    int num_dev_bits,
    const unsigned char *dev_buf,
    size_t dev_len_per_block,
    unsigned char *out_block)
{
    if (!base_block || !dev_positions || !dev_buf || !out_block)
        return 0;

    /* 우선 out_block = base_block */
    memcpy(out_block, base_block, block_bytes);

    size_t block_bits = block_bytes * 8;
    (void)dev_len_per_block; /* 필요시 추가 검증용으로 사용 가능 */

    for (int i = 0; i < num_dev_bits; ++i)
    {
        int bit_pos = dev_positions[i];
        if (bit_pos < 0 || (size_t)bit_pos >= block_bits)
        {
            fprintf(stderr,
                    "merge_base_and_deviation_by_pos: invalid bit_pos=%d (block_bits=%zu)\n",
                    bit_pos, block_bits);
            return 0;
        }

        uint8_t bit = get_dev_bit(dev_buf, (size_t)i);
        set_block_bit(out_block, (size_t)bit_pos, bit);
    }

    return (size_t)num_dev_bits;
}

/* ------------------------------------------------------------
 * 하나의 segment(출력 파일 조각)를 DDP1 파일로 쓰는 helper
 *
 *   - base_out    : 기본 출력 파일 이름
 *   - segment_idx : 0 → base_out,
 *                   1 → base_out.seg1,
 *                   2 → base_out.seg2, ...
 * ------------------------------------------------------------ */
static int write_ddp1_segment(const char *base_out,
                              int segment_idx,
                              size_t block_bytes,
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
    {
        /* 쓸 블록이 없으면 아무것도 안 함 */
        return 0;
    }

    char filename[1024];
    if (segment_idx == 0)
    {
        snprintf(filename, sizeof(filename), "%s", base_out);
    }
    else
    {
        snprintf(filename, sizeof(filename), "%s.seg%d", base_out, segment_idx);
    }

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
        !write_u32_le(fp, num_fields_u32) ||
        !write_u32_le(fp, dict_size_u32) ||
        !write_u32_le(fp, num_blocks_u32) ||
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

    /* dictionary blocks */
    for (int i = 0; i < dict->size; ++i)
    {
        if (fwrite(dict->blocks[i], 1, block_bytes, fp) != block_bytes)
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

    /* deviation stream (bit-packed) */
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
            "dev_len_per_block=%zu (bytes), dev_pos_count=%d (bits)\n",
            segment_idx, filename, num_blocks_segment, dict->size,
            dev_len_per_block, dev_pos_count);

    return 0;
}

/* ------------------------------------------------------------
 * multi-layout 압축 (DDP1)
 *
 * 포맷 (각 segment 파일마다):
 *  magic: 'D','D','P','1'
 *  u32: block_bytes        (한 블록의 총 바이트 수)
 *  u32: num_fields         (블록 내부 field 개수)
 *  u32: dict_size          (사전에 저장된 base 블록 수)
 *  u32: num_blocks         (이 segment 안의 블록 수)
 *  u32: dev_pos_count      (# deviation bit positions)
 *  u32: dev_len_per_block  (deviation bitstream 길이, bytes)
 *  u32[num_fields]:  field_sizes  (각 field의 byte 수)
 *  u32[dev_pos_count]: dev_positions (block 내 deviation bit 오프셋들)
 *
 *  [dictionary]: dict_size * block_bytes bytes
 *  [block_ids]:  num_blocks * 1 byte (uint8_t, 0 ~ 255)
 *  [deviation]:  num_blocks * dev_len_per_block bytes (bit-packed)
 *
 *  - dict_size가 255에 도달하면:
 *      → 현재까지의 block들을 하나의 segment 파일로 flush
 *      → dictionary를 비우고, 새로운 segment 파일에 이어서 기록
 * ------------------------------------------------------------ */

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

    /* deviation 길이 계산 (전역 g_dev_positions 사용, bit-count 기반) */
    if (g_num_dev_positions < 0)
    {
        fprintf(stderr, "compress_file_multi: invalid g_num_dev_positions\n");
        return 1;
    }
    size_t dev_len_per_block = compute_dev_len_from_positions(g_num_dev_positions);

    /* dev_positions가 block_bits 범위 안에 있는지 확인 */
    size_t block_bits = block_bytes * 8;
    for (int i = 0; i < g_num_dev_positions; ++i)
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

    /* 입력 파일 열기 */
    FILE *fin = fopen(input_filename, "rb");
    if (!fin)
    {
        perror("fopen input");
        return 1;
    }

    if (fseek(fin, 0, SEEK_END) != 0)
    {
        perror("fseek end");
        fclose(fin);
        return 1;
    }
    long file_size = ftell(fin);
    if (file_size < 0)
    {
        perror("ftell");
        fclose(fin);
        return 1;
    }
    if (fseek(fin, 0, SEEK_SET) != 0)
    {
        perror("fseek set");
        fclose(fin);
        return 1;
    }

    size_t nbytes = (size_t)file_size;
    if (nbytes < block_bytes)
    {
        fprintf(stderr, "Input file too small for one multi-layout block\n");
        fclose(fin);
        return 1;
    }

    size_t num_blocks_total = nbytes / block_bytes;
    if (num_blocks_total == 0)
    {
        fprintf(stderr, "No full blocks found (multi-layout)\n");
        fclose(fin);
        return 1;
    }

    size_t used_bytes = num_blocks_total * block_bytes;
    if (used_bytes < nbytes)
    {
        fprintf(stderr,
                "Warning: last %zu bytes are ignored (not enough to fill a block)\n",
                (nbytes - used_bytes));
    }

    /* 전체 스트림에 대한 block_ids / deviation stream 버퍼 */
    uint8_t *block_ids = (uint8_t *)malloc(sizeof(uint8_t) * num_blocks_total);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        fclose(fin);
        return 1;
    }

    size_t dev_total_bytes = num_blocks_total * dev_len_per_block;
    unsigned char *dev_stream = NULL;
    if (dev_len_per_block > 0)
    {
        dev_stream = (unsigned char *)malloc(dev_total_bytes);
        if (!dev_stream)
        {
            fprintf(stderr, "Failed to allocate dev_stream (multi)\n");
            free(block_ids);
            fclose(fin);
            return 1;
        }
    }

    Dictionary dict;
    dict_init(&dict, block_bytes);

    unsigned char *block_buf = (unsigned char *)malloc(block_bytes);
    unsigned char *dev_buf = (dev_len_per_block > 0)
                                 ? (unsigned char *)malloc(dev_len_per_block)
                                 : NULL;

    if (!block_buf || (dev_len_per_block > 0 && !dev_buf))
    {
        fprintf(stderr, "Failed to allocate block/dev buffer (multi)\n");
        free(block_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    /* --- BitStats 초기화 (비트 변동 통계용) --- */
    BitStats global_stats;
    BitStats segment_stats;
    if (bitstats_init(&global_stats, block_bits) != 0 ||
        bitstats_init(&segment_stats, block_bits) != 0)
    {
        fprintf(stderr, "Failed to init BitStats\n");
        free(block_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    /* raw 블록을 따로 보관해서 deviation 추출로 block_buf가 바뀌어도
     * 비트 통계는 raw 기준으로 계산.
     */
    unsigned char *raw_block_buf = (unsigned char *)malloc(block_bytes);
    if (!raw_block_buf)
    {
        fprintf(stderr, "Failed to allocate raw_block_buf\n");
        bitstats_free(&global_stats);
        bitstats_free(&segment_stats);
        free(block_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    /* segment 관리 변수 */
    int    segment_idx          = 0;    /* 0 -> output.ddp, 1 -> output.ddp.seg1 ... */
    size_t global_b             = 0;    /* 전체 block index */
    size_t segment_start_block  = 0;    /* 이 segment가 시작되는 global block index */
    size_t segment_block_count  = 0;    /* 이 segment 안에 실제로 포함된 block 수 */

    while (global_b < num_blocks_total)
    {
        /* block 하나 읽기 */
        size_t n = fread(block_buf, 1, block_bytes, fin);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read block %zu (multi)\n", global_b);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            free(raw_block_buf);
            bitstats_free(&global_stats);
            bitstats_free(&segment_stats);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        /* deviation 추출 전에 raw 복사 */
        memcpy(raw_block_buf, block_buf, block_bytes);

        /* deviation 추출 (bit packed) */
        if (dev_len_per_block > 0)
        {
            size_t used_dev = extract_base_and_deviation_by_pos(
                block_buf,
                block_bytes,
                g_dev_positions,
                g_num_dev_positions,
                dev_buf,
                dev_len_per_block);

            if (used_dev != dev_len_per_block)
            {
                fprintf(stderr,
                        "extract_base_and_deviation_by_pos: used_dev=%zu, expected=%zu\n",
                        used_dev, dev_len_per_block);
                free(block_buf);
                free(dev_buf);
                free(dev_stream);
                free(block_ids);
                free(raw_block_buf);
                bitstats_free(&global_stats);
                bitstats_free(&segment_stats);
                dict_free(&dict);
                fclose(fin);
                return 1;
            }

            memcpy(dev_stream + global_b * dev_len_per_block,
                   dev_buf,
                   dev_len_per_block);
        }

        /* dictionary lookup / insert
         * - 새로운 base가 들어왔는데 dict.size == 255라면
         *   → 지금까지 모인 block들을 한 segment 파일로 flush 후
         *   → dictionary 비우고 다음 segment 시작
         *
         *   이때 현재 블록(global_b)은 "다음 segment의 첫 블록"이 된다.
         */
        int idx = dict_find(&dict, block_buf);
        if (idx == -1)
        {
            if (dict.size >= 255)
            {
                /* 현재 segment flush (이전 블록들만 포함) */
                size_t blocks_in_segment = segment_block_count;
                if (blocks_in_segment > 0)
                {
                    const uint8_t *segment_block_ids =
                        block_ids + segment_start_block;
                    const unsigned char *segment_dev_stream = NULL;

                    if (dev_len_per_block > 0 && dev_stream)
                    {
                        segment_dev_stream =
                            dev_stream + segment_start_block * dev_len_per_block;
                    }

                    if (write_ddp1_segment(output_filename,
                                           segment_idx,
                                           block_bytes,
                                           num_fields,
                                           field_sizes,
                                           g_num_dev_positions,
                                           g_dev_positions,
                                           dev_len_per_block,
                                           &dict,
                                           segment_block_ids,
                                           segment_dev_stream,
                                           blocks_in_segment) != 0)
                    {
                        fprintf(stderr, "Failed to write segment %d\n",
                                segment_idx);
                        free(block_buf);
                        free(dev_buf);
                        free(dev_stream);
                        free(block_ids);
                        free(raw_block_buf);
                        bitstats_free(&global_stats);
                        bitstats_free(&segment_stats);
                        dict_free(&dict);
                        fclose(fin);
                        return 1;
                    }

                    /* 세그먼트별 비트 변동 통계 출력 */
                    char label[64];
                    snprintf(label, sizeof(label), "segment %d", segment_idx);
                    bitstats_print(&segment_stats, label, stderr);
                    bitstats_reset(&segment_stats);
                }

                /* 새 segment 시작 */
                dict_free(&dict);
                dict_init(&dict, block_bytes);

                segment_idx++;
                segment_start_block = global_b;   /* 현재 블록이 새 segment의 첫 블록 */
                segment_block_count = 0;
            }

            idx = dict_add(&dict, block_buf);
            if (idx < 0)
            {
                fprintf(stderr, "dict_add failed\n");
                free(block_buf);
                free(dev_buf);
                free(dev_stream);
                free(block_ids);
                free(raw_block_buf);
                bitstats_free(&global_stats);
                bitstats_free(&segment_stats);
                dict_free(&dict);
                fclose(fin);
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
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            free(raw_block_buf);
            bitstats_free(&global_stats);
            bitstats_free(&segment_stats);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        block_ids[global_b] = (uint8_t)idx;

        /* --- 비트 변동 통계 업데이트 (raw 데이터 기준) ---
         * global_stats: 전체 파일 기준
         * segment_stats: 현재 segment 기준 (segment flush 시 출력+reset)
         */
        bitstats_update(&global_stats,  raw_block_buf, block_bytes);
        bitstats_update(&segment_stats, raw_block_buf, block_bytes);

        segment_block_count++;
        global_b++;
    }

    fclose(fin);

    /* 마지막 segment flush */
    if (segment_block_count > 0)
    {
        const uint8_t *segment_block_ids =
            block_ids + segment_start_block;
        const unsigned char *segment_dev_stream = NULL;
        if (dev_len_per_block > 0 && dev_stream)
        {
            segment_dev_stream =
                dev_stream + segment_start_block * dev_len_per_block;
        }

        if (write_ddp1_segment(output_filename,
                               segment_idx,
                               block_bytes,
                               num_fields,
                               field_sizes,
                               g_num_dev_positions,
                               g_dev_positions,
                               dev_len_per_block,
                               &dict,
                               segment_block_ids,
                               segment_dev_stream,
                               segment_block_count) != 0)
        {
            fprintf(stderr, "Failed to write final segment %d\n", segment_idx);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            free(raw_block_buf);
            bitstats_free(&global_stats);
            bitstats_free(&segment_stats);
            dict_free(&dict);
            return 1;
        }

        /* 마지막 세그먼트 비트 통계 출력 */
        char label[64];
        snprintf(label, sizeof(label), "segment %d", segment_idx);
        bitstats_print(&segment_stats, label, stderr);
        /* segment_stats는 곧 free될 예정이라 reset은 필요 없음 */
    }

    free(block_buf);
    free(dev_buf);
    free(dev_stream);
    free(block_ids);
    free(raw_block_buf);
    dict_free(&dict);

    /* 전체(global) 비트 변동 통계 출력 */
    bitstats_print(&global_stats, "global", stderr);
    bitstats_free(&global_stats);
    bitstats_free(&segment_stats);

    fprintf(stderr,
            "Compressed (DDP1 multi, segmented, bit-deviation): "
            "used_bytes=%zu, block_bytes=%zu, total_blocks=%zu, "
            "dev_len_per_block=%zu (bytes), dev_pos_count=%d (bits), segments=%d\n",
            used_bytes, block_bytes, num_blocks_total,
            dev_len_per_block, g_num_dev_positions, segment_idx + 1);

    return 0;
}

/* ============================================================
 * Decompression
 *   - 한 번에 하나의 segment 파일만 복원
 *   - 즉, output.ddp, output.ddp.seg1, ... 각각 따로 decompress_file() 호출
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
        !read_u32_le(fp, &num_fields_u32) ||
        !read_u32_le(fp, &dict_size_u32) ||
        !read_u32_le(fp, &num_blocks_u32) ||
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
    int    dev_pos_count     = (int)dev_pos_count_u32;   /* #bits */
    size_t dev_len_per_block = (size_t)dev_len_u32;      /* bytes */

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

        size_t block_bits = block_bytes * 8;
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

    /* dictionary 읽기 */
    Dictionary dict;
    dict_init(&dict, block_bytes);

    for (size_t i = 0; i < dict_size; ++i)
    {
        unsigned char *buf = (unsigned char *)malloc(block_bytes);
        if (!buf)
        {
            fprintf(stderr, "Failed to allocate block buffer (dict read, multi)\n");
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        size_t n = fread(buf, 1, block_bytes, fp);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read dictionary block %zu (multi)\n", i);
            free(buf);
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
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        block_ids[b] = (uint8_t)c;
    }

    /* deviation stream 읽기 (bit-packed) */
    size_t dev_total_bytes = num_blocks * dev_len_per_block;
    unsigned char *dev_stream = NULL;
    if (dev_total_bytes > 0)
    {
        dev_stream = (unsigned char *)malloc(dev_total_bytes);
        if (!dev_stream)
        {
            fprintf(stderr, "Failed to allocate dev_stream (multi dec)\n");
            free(block_ids);
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
            free(dev_positions);
            free(field_sizes);
            dict_free(&dict);
            return 1;
        }

        const unsigned char *base_block = dict.blocks[id];
        const unsigned char *dev_ptr = (dev_stream && dev_len_per_block > 0)
                                           ? (dev_stream + b * dev_len_per_block)
                                           : NULL;

        if (dev_ptr && dev_pos_count > 0)
        {
            size_t used_dev = merge_base_and_deviation_by_pos(
                base_block,
                block_bytes,
                dev_positions,
                dev_pos_count,
                dev_ptr,
                dev_len_per_block,
                tmp);

            if (used_dev != (size_t)dev_pos_count)
            {
                fprintf(stderr,
                        "merge used_dev=%zu != dev_pos_count=%d (multi dec)\n",
                        used_dev, dev_pos_count);
                free(out);
                free(tmp);
                free(dev_stream);
                free(block_ids);
                free(dev_positions);
                free(field_sizes);
                dict_free(&dict);
                return 1;
            }
        }
        else
        {
            memcpy(tmp, base_block, block_bytes);
        }

        size_t offset = b * block_bytes;
        if (offset + block_bytes > total_bytes)
        {
            fprintf(stderr, "Output buffer overflow risk (multi dec)\n");
            free(out);
            free(tmp);
            free(dev_stream);
            free(block_ids);
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
    free(dev_positions);
    free(field_sizes);
    dict_free(&dict);

    return ret;
}
