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

static inline uint8_t get_block_bit(const unsigned char *buf, size_t bit_pos)
{
    size_t byte_idx = bit_pos / 8;
    int bit_idx = (int)(bit_pos % 8); /* LSB-first */
    return (uint8_t)((buf[byte_idx] >> bit_idx) & 1u);
}

static inline void set_block_bit(unsigned char *buf, size_t bit_pos, uint8_t v)
{
    size_t byte_idx = bit_pos / 8;
    int bit_idx = (int)(bit_pos % 8);
    if (v)
        buf[byte_idx] |= (unsigned char)(1u << bit_idx);
    else
        buf[byte_idx] &= (unsigned char)~(1u << bit_idx);
}

static inline uint8_t get_dev_bit(const unsigned char *buf, size_t bit_idx)
{
    size_t byte_idx = bit_idx / 8;
    int bit_in = (int)(bit_idx % 8);
    return (uint8_t)((buf[byte_idx] >> bit_in) & 1u);
}

static inline void set_dev_bit(unsigned char *buf, size_t bit_idx, uint8_t v)
{
    size_t byte_idx = bit_idx / 8;
    int bit_in = (int)(bit_idx % 8);
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
 * ============================================================ */

static size_t compute_dev_len_from_positions(int num_bits)
{
    if (num_bits <= 0)
        return 0;
    return (size_t)((num_bits + 7) / 8); /* ceil(num_bits/8) */
}

/* block_buf에서 선택된 비트를 deviation bitstream(dev_buf)에 모으고,
 * block_buf의 해당 비트는 0으로 만들어 base만 남긴다.
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

/* base_block + dev_buf(bitstream) → out_block (bit 위치 기반 복원) */
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
    (void)dev_len_per_block; /* 필요시 추가 검증용 */

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
 * bitstats 결과에서 상위 dev_top_n 비트 선택
 * ------------------------------------------------------------ */

typedef struct
{
    size_t bit_index;
    uint64_t count;
} BitRank;

static int cmp_bitrank_desc(const void *a, const void *b)
{
    const BitRank *pa = (const BitRank *)a;
    const BitRank *pb = (const BitRank *)b;

    if (pa->count < pb->count)
        return 1;
    if (pa->count > pb->count)
        return -1;
    if (pa->bit_index > pb->bit_index)
        return 1;
    if (pa->bit_index < pb->bit_index)
        return -1;
    return 0;
}

/* dev_top_n 개수만큼 deviation 비트 위치를 자동 선택
 *  - out_positions: malloc된 int 배열을 돌려줌 (호출자가 free)
 *  - 반환값: 실제 선택된 개수 (<= dev_top_n)
 */
static size_t init_dev_positions_with_bitstats(const BitStats *bs,
                                               int dev_top_n,
                                               int **out_positions)
{
    if (!bs || !out_positions)
        return 0;

    size_t num_bits = bs->num_bits;
    if (num_bits == 0)
        return 0;

    if (dev_top_n <= 0 || (size_t)dev_top_n > num_bits)
        dev_top_n = (int)num_bits;

    BitRank *arr = (BitRank *)malloc(num_bits * sizeof(BitRank));
    if (!arr)
        return 0;

    for (size_t i = 0; i < num_bits; ++i)
    {
        arr[i].bit_index = i;
        arr[i].count = bs->change_counts[i];
    }

    qsort(arr, num_bits, sizeof(BitRank), cmp_bitrank_desc);

    int *positions = (int *)malloc((size_t)dev_top_n * sizeof(int));
    if (!positions)
    {
        free(arr);
        return 0;
    }

    for (int k = 0; k < dev_top_n; ++k)
    {
        positions[k] = (int)arr[k].bit_index;
    }

    free(arr);
    *out_positions = positions;
    return (size_t)dev_top_n;
}

/* ------------------------------------------------------------
 * 다음 segment에서 사용할 deviation 비트 개수 결정 (online policy hook)
 *
 *  - next_segment_idx : 현재 segment가 flush된 뒤 시작할 segment 번호
 *                       (0 기반; 1이면 첫 번째 dev segment)
 *  - block_bits       : 한 block 당 비트 수 (block_bytes * 8)
 *  - prev_seg_blocks  : 방금 flush한 segment 안의 block 개수
 *  - prev_bs          : 방금 flush한 segment 동안 측정된 BitStats
 *
 *  이 함수 안의 로직만 바꾸면 online 학습 정책을 자유롭게 바꿀 수 있다.
 *  (지금은 "예시용"으로 매우 단순한 heuristic을 넣어 두었다.)
 */
static int decide_dev_pos_count_for_segment(int next_segment_idx,
                                            size_t block_bits,
                                            size_t prev_seg_blocks,
                                            const BitStats *prev_bs)
{
    /* --- 이전 segment dev bit 기억용 (simple state) --- */
    static int  last_dev_bits = -1;  /* 직전 segment에서 실제 사용한 dev_bits */
    static int  state_inited  = 0;

    /* ---- 통계값 계산: 나중에 튜닝용으로도 쓰려고 그대로 유지 ---- */
    size_t   num_bits        = 0;
    size_t   hot_bits        = 0;
    uint64_t total_changes   = 0;
    double   hot_ratio       = 0.0;
    double   avg_changes_per_block = 0.0;
    double   change_rate     = 0.0;

    if (prev_bs)
    {
        num_bits = prev_bs->num_bits;
        if (num_bits > 0 && prev_seg_blocks > 0)
        {
            for (size_t i = 0; i < num_bits; ++i)
            {
                uint64_t c = prev_bs->change_counts[i];
                total_changes += c;
                if (c > 0)
                    hot_bits++;
            }

            hot_ratio = (double)hot_bits / (double)num_bits;  /* 0.0 ~ 1.0 */

            avg_changes_per_block =
                (double)total_changes / (double)prev_seg_blocks; /* 0 이상 */

            change_rate = avg_changes_per_block / (double)num_bits;
            if (change_rate > 1.0)
                change_rate = 1.0;
        }
    }

    /* 디버그용 출력 (정책 튜닝용) */
    fprintf(stderr,
            "[decide_dev_pos_count_for_segment] next_seg=%d, "
            "block_bits=%zu, prev_seg_blocks=%zu, "
            "hot_bits=%zu, hot_ratio=%.4f, "
            "avg_changes_per_block=%.4f, change_rate=%.4f\n",
            next_segment_idx, block_bits, prev_seg_blocks,
            hot_bits, hot_ratio, avg_changes_per_block, change_rate);

    if (block_bits == 0)
        return 0;

    size_t target_bits = 0;

    /* ---------- segment 0: 완전 dedup ---------- */
    if (next_segment_idx == 0)
    {
        target_bits = 0;
        last_dev_bits = 0;
        state_inited = 1;
    }
    else
    {
        /* ===== 1) hot_bits 기반 목표 dev bit 계산 =====
         *
         *   - base_hot = hot_bits (실제로 변한 비트 수)
         *   - 너무 작으면 최소값(min_bits)로 올려주고,
         *   - 너무 크면 최대값(max_bits)에서 잘라줌.
         *
         *   튜닝 포인트:
         *     - min_frac: block_bits 대비 dev 최소 비율
         *     - max_frac: block_bits 대비 dev 최대 비율
         */

        double min_frac = 1.0 / 8.0;   /* 최소 1/8 정도는 허용 (예: 128bit → 16bit) */
        double max_frac = 1.0 / 2.0;   /* 최대 1/2 까지 (예: 128bit → 64bit) */

        size_t min_bits = (size_t)(block_bits * min_frac + 0.5);
        size_t max_bits = (size_t)(block_bits * max_frac + 0.5);
        if (min_bits < 1) min_bits = 1;
        if (max_bits > block_bits) max_bits = block_bits;

        size_t base_hot = hot_bits;
        if (base_hot == 0)
        {
            /* 변하는 비트가 하나도 안 잡힌 경우:
             * 그냥 최소 dev만 써보자
             */
            base_hot = min_bits;
        }

        /* hot_bits에 여유 계수 살짝 곱해도 됨 (ex: 1.2~1.5) */
        double overshoot = 1.2;  /* 튜닝포인트: hot_bits보다 조금 크게 쓸지 */
        size_t base_bits = (size_t)( (double)base_hot * overshoot + 0.5 );

        if (base_bits < min_bits) base_bits = min_bits;
        if (base_bits > max_bits) base_bits = max_bits;

        /* ===== 2) segment index / 이전 dev와의 차이를 이용해서 완만하게 이동 ===== */

        if (!state_inited || last_dev_bits < 0)
        {
            /* 첫 dev segment: base_bits 그대로 사용 */
            target_bits = base_bits;
            state_inited = 1;
        }
        else
        {
            /* 이전 dev와 base_bits의 차이가 너무 크면 천천히 보정 */
            int step_limit = 8;  /* 한 segment당 최대 ±8bit만 움직이게 (튜닝 가능) */

            int diff = (int)base_bits - last_dev_bits;
            if (diff > step_limit)
                diff = step_limit;
            else if (diff < -step_limit)
                diff = -step_limit;

            int new_bits = last_dev_bits + diff;

            if (new_bits < (int)min_bits) new_bits = (int)min_bits;
            if ((size_t)new_bits > max_bits) new_bits = (int)max_bits;

            target_bits = (size_t)new_bits;
        }

        last_dev_bits = (int)target_bits;
    }

    /* 최종 safety clamp */
    if (target_bits > block_bits)
        target_bits = block_bits;
    if (target_bits == 0 && next_segment_idx >= 1)
        target_bits = 1;

    fprintf(stderr,
            "[policy] next_seg=%d -> dev_bits=%zu (last_dev_bits=%d)\n",
            next_segment_idx, target_bits, last_dev_bits);

    return (int)target_bits;
}



/* ------------------------------------------------------------
 * 하나의 segment(출력 파일 조각)를 DDP1 파일로 쓰는 helper
 *   - 첫 segment(0)는 dev_pos_count=0 / dev_len_per_block=0 으로 호출 가능
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
        return 0;
    }

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

    uint32_t block_bytes_u32 = (uint32_t)block_bytes;
    uint32_t num_fields_u32 = (uint32_t)num_fields;
    uint32_t dict_size_u32 = (uint32_t)dict->size;
    uint32_t num_blocks_u32 = (uint32_t)num_blocks_segment;
    uint32_t dev_pos_count_u32 = (uint32_t)dev_pos_count; /* #bits */
    uint32_t dev_len_u32 = (uint32_t)dev_len_per_block;   /* bytes */

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
 * - pass1: 전체 파일 bitstats 수집
 * - pass2:
 *    * segment 0: dev 없음 (순수 dedup)
 *    * segment 1~: bitstats 기반 top-N dev 적용
 * ------------------------------------------------------------ */

int compress_file(const char *input_filename,
                  const char *output_filename,
                  int num_fields,
                  const int *field_sizes,
                  int dev_top_n)
{
    (void)dev_top_n; /* 현재 구현에서는 online policy 안에서만 dev 개수를 결정하므로
                        인자는 사용하지 않는다. 필요하면 decide_dev_pos_count_for_segment()
                        안에서 활용하도록 변경 가능. */

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

    /* 입력 파일 열기 및 block 수 계산 */
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

    size_t block_bits = block_bytes * 8;

    /* bitstats: 전체(global) + segment 단위 */
    BitStats *bs_global = bitstats_create(block_bits);
    BitStats *bs_seg = bitstats_create(block_bits);
    if (!bs_global || !bs_seg)
    {
        fprintf(stderr, "Failed to create BitStats (global/segment)\n");
        if (bs_global)
            bitstats_free(bs_global);
        if (bs_seg)
            bitstats_free(bs_seg);
        fclose(fin);
        return 1;
    }

    unsigned char *block_buf = (unsigned char *)malloc(block_bytes);
    if (!block_buf)
    {
        fprintf(stderr, "Failed to allocate block_buf\n");
        bitstats_free(bs_global);
        bitstats_free(bs_seg);
        fclose(fin);
        return 1;
    }

    /* 전체 스트림에 대한 block_ids (segment 나뉘어도 같은 배열 사용) */
    uint8_t *block_ids = (uint8_t *)malloc(sizeof(uint8_t) * num_blocks_total);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        free(block_buf);
        bitstats_free(bs_global);
        bitstats_free(bs_seg);
        fclose(fin);
        return 1;
    }

    /* segment 별 deviation stream 버퍼 */
    unsigned char *seg_dev_stream = NULL;
    size_t seg_dev_capacity = 0;
    size_t seg_dev_size = 0;

    /* 현재 segment에서 사용하는 deviation bit 정보 */
    int cur_dev_pos_count = 0;
    int *cur_dev_positions = NULL;
    size_t cur_dev_len = 0; /* bytes per block */

    /* deviation 추출용 임시 버퍼 (크기가 바뀔 수 있으므로 capacity 관리) */
    unsigned char *dev_buf = NULL;
    size_t dev_buf_capacity = 0;

    Dictionary dict;
    dict_init(&dict, block_bytes);

    /* segment 관리 변수 */
    int segment_idx = 0;            /* 0 -> 첫 segment (기본: dev 없음) */
    size_t global_b = 0;            /* 전체 block index */
    size_t segment_start_block = 0; /* 이 segment 시작 global index */
    size_t segment_block_count = 0; /* 이 segment 안 block 수 */

    /* 메인 루프: 한 번만 돌면서 bitstats + 압축 동시에 수행 */
    while (global_b < num_blocks_total)
    {
        size_t n = fread(block_buf, 1, block_bytes, fin);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read block %zu (multi, single-pass)\n",
                    global_b);
            free(dev_buf);
            free(seg_dev_stream);
            free(block_ids);
            free(block_buf);
            if (cur_dev_positions)
                free(cur_dev_positions);
            bitstats_free(bs_global);
            bitstats_free(bs_seg);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        /* 전체 bit 통계 (전체 파일 기준) */
        bitstats_update_block(bs_global, block_buf, block_bytes);

        /* dictionary lookup */
        int idx = dict_find(&dict, block_buf);

        if (idx == -1 && dict.size >= 255)
        {
            /* 현재 segment flush */
            size_t blocks_in_segment = segment_block_count;
            if (blocks_in_segment > 0)
            {
                const uint8_t *segment_block_ids =
                    block_ids + segment_start_block;

                const unsigned char *segment_dev_stream_ptr = NULL;
                if (cur_dev_len > 0 && seg_dev_size > 0)
                    segment_dev_stream_ptr = seg_dev_stream;

                if (write_ddp1_segment(output_filename,
                                       segment_idx,
                                       block_bytes,
                                       num_fields,
                                       field_sizes,
                                       cur_dev_pos_count,
                                       cur_dev_positions,
                                       cur_dev_len,
                                       &dict,
                                       segment_block_ids,
                                       segment_dev_stream_ptr,
                                       blocks_in_segment) != 0)
                {
                    fprintf(stderr, "Failed to write segment %d\n", segment_idx);
                    free(dev_buf);
                    free(seg_dev_stream);
                    free(block_ids);
                    free(block_buf);
                    if (cur_dev_positions)
                        free(cur_dev_positions);
                    bitstats_free(bs_global);
                    bitstats_free(bs_seg);
                    dict_free(&dict);
                    fclose(fin);
                    return 1;
                }
            }

            /* 다음 segment에서 사용할 deviation bit 개수 결정
             *  - 현재 segment의 bitstats(bs_seg)와 block 개수(segment_block_count)를 사용
             */
            int next_segment_idx = segment_idx + 1;
            int next_dev_pos_count = decide_dev_pos_count_for_segment(
                next_segment_idx,
                block_bits,
                segment_block_count,
                bs_seg);

            /* 이전 dev 정보 정리 후 새 dev 정보 설정 */
            if (cur_dev_positions)
            {
                free(cur_dev_positions);
                cur_dev_positions = NULL;
            }
            cur_dev_pos_count = 0;
            cur_dev_len = 0;

            if (next_dev_pos_count > 0)
            {
                size_t selected =
                    init_dev_positions_with_bitstats(bs_seg,
                                                     next_dev_pos_count,
                                                     &cur_dev_positions);
                cur_dev_pos_count = (int)selected;
                cur_dev_len = compute_dev_len_from_positions(cur_dev_pos_count);

                fprintf(stderr,
                        "[segment %d -> %d] next_dev_pos_count=%d (selected=%zu), "
                        "cur_dev_len=%zu bytes\n",
                        segment_idx, next_segment_idx,
                        next_dev_pos_count, selected, cur_dev_len);
            }
            else
            {
                fprintf(stderr,
                        "[segment %d -> %d] next_dev_pos_count=0 (pure dedup)\n",
                        segment_idx, next_segment_idx);
            }

            /* segment bitstats 초기화 (다음 segment 통계 전용) */
            bitstats_reset(bs_seg);

            /* dictionary 초기화 (새 segment) */
            dict_free(&dict);
            dict_init(&dict, block_bytes);

            /* segment deviation stream 버퍼 초기화 */
            if (seg_dev_stream)
            {
                free(seg_dev_stream);
                seg_dev_stream = NULL;
            }
            seg_dev_capacity = 0;
            seg_dev_size = 0;

            /* segment 인덱스 및 범위 갱신 */
            segment_idx++;
            segment_start_block = global_b;
            segment_block_count = 0;

            /* idx는 여전히 -1 이므로, 아래에서 새 dict에 추가됨 */
        }

        /* dictionary에 base 삽입 (필요 시) */
        if (idx == -1)
        {
            idx = dict_add(&dict, block_buf);
            if (idx < 0)
            {
                fprintf(stderr, "dict_add failed\n");
                free(dev_buf);
                free(seg_dev_stream);
                free(block_ids);
                free(block_buf);
                if (cur_dev_positions)
                    free(cur_dev_positions);
                bitstats_free(bs_global);
                bitstats_free(bs_seg);
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
            free(dev_buf);
            free(seg_dev_stream);
            free(block_ids);
            free(block_buf);
            if (cur_dev_positions)
                free(cur_dev_positions);
            bitstats_free(bs_global);
            bitstats_free(bs_seg);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        /* 현재 segment에 대한 bitstats 업데이트
         *  - dictionary segmentation(flush) 후에 호출되므로
         *    bs_seg에는 항상 "현재 segment"의 block들만 쌓인다.
         */
        bitstats_update_block(bs_seg, block_buf, block_bytes);

        /* deviation 사용 여부: cur_dev_pos_count > 0 이면 사용 */
        if (cur_dev_pos_count > 0 && cur_dev_len > 0)
        {
            /* dev_buf 크기 확보 */
            if (dev_buf_capacity < cur_dev_len)
            {
                unsigned char *tmp = (unsigned char *)realloc(dev_buf, cur_dev_len);
                if (!tmp)
                {
                    fprintf(stderr, "Failed to realloc dev_buf\n");
                    free(dev_buf);
                    free(seg_dev_stream);
                    free(block_ids);
                    free(block_buf);
                    if (cur_dev_positions)
                        free(cur_dev_positions);
                    bitstats_free(bs_global);
                    bitstats_free(bs_seg);
                    dict_free(&dict);
                    fclose(fin);
                    return 1;
                }
                dev_buf = tmp;
                dev_buf_capacity = cur_dev_len;
            }

            size_t used_dev = extract_base_and_deviation_by_pos(
                block_buf,
                block_bytes,
                cur_dev_positions,
                cur_dev_pos_count,
                dev_buf,
                cur_dev_len);

            if (used_dev != cur_dev_len)
            {
                fprintf(stderr,
                        "extract_base_and_deviation_by_pos: used_dev=%zu, expected=%zu\n",
                        used_dev, cur_dev_len);
                free(dev_buf);
                free(seg_dev_stream);
                free(block_ids);
                free(block_buf);
                if (cur_dev_positions)
                    free(cur_dev_positions);
                bitstats_free(bs_global);
                bitstats_free(bs_seg);
                dict_free(&dict);
                fclose(fin);
                return 1;
            }

            /* segment deviation stream에 dev_buf append */
            size_t needed = seg_dev_size + cur_dev_len;
            if (needed > seg_dev_capacity)
            {
                size_t new_cap = (seg_dev_capacity == 0) ? (cur_dev_len * 16) : (seg_dev_capacity * 2);
                if (new_cap < needed)
                    new_cap = needed;

                unsigned char *tmp = (unsigned char *)realloc(seg_dev_stream, new_cap);
                if (!tmp)
                {
                    fprintf(stderr, "Failed to realloc seg_dev_stream\n");
                    free(dev_buf);
                    free(seg_dev_stream);
                    free(block_ids);
                    free(block_buf);
                    if (cur_dev_positions)
                        free(cur_dev_positions);
                    bitstats_free(bs_global);
                    bitstats_free(bs_seg);
                    dict_free(&dict);
                    fclose(fin);
                    return 1;
                }
                seg_dev_stream = tmp;
                seg_dev_capacity = new_cap;
            }

            memcpy(seg_dev_stream + seg_dev_size, dev_buf, cur_dev_len);
            seg_dev_size += cur_dev_len;
        }

        /* block id 기록 및 segment 카운터 갱신 */
        block_ids[global_b] = (uint8_t)idx;
        segment_block_count++;
        global_b++;
    } /* while */

    fclose(fin);

    /* 마지막 segment flush */
    if (segment_block_count > 0)
    {
        size_t blocks_in_segment = segment_block_count;
        const uint8_t *segment_block_ids =
            block_ids + segment_start_block;

        const unsigned char *segment_dev_stream_ptr = NULL;
        if (cur_dev_len > 0 && seg_dev_size > 0)
            segment_dev_stream_ptr = seg_dev_stream;

        if (write_ddp1_segment(output_filename,
                               segment_idx,
                               block_bytes,
                               num_fields,
                               field_sizes,
                               cur_dev_pos_count,
                               cur_dev_positions,
                               cur_dev_len,
                               &dict,
                               segment_block_ids,
                               segment_dev_stream_ptr,
                               blocks_in_segment) != 0)
        {
            fprintf(stderr, "Failed to write final segment %d\n", segment_idx);
            free(dev_buf);
            free(seg_dev_stream);
            free(block_ids);
            free(block_buf);
            if (cur_dev_positions)
                free(cur_dev_positions);
            bitstats_free(bs_global);
            bitstats_free(bs_seg);
            dict_free(&dict);
            return 1;
        }
    }

    /* 전체 파일 기준 bitstats 결과를 한번 찍어본다 (디버깅용) */
    bitstats_print_sorted(stderr, bs_global, 64);

    free(dev_buf);
    if (seg_dev_stream)
        free(seg_dev_stream);
    free(block_ids);
    free(block_buf);
    if (cur_dev_positions)
        free(cur_dev_positions);
    bitstats_free(bs_global);
    bitstats_free(bs_seg);
    dict_free(&dict);

    fprintf(stderr,
            "Compressed (DDP1 multi, segmented, online dev-top-N): "
            "used_bytes=%zu, block_bytes=%zu, total_blocks=%zu, "
            "segments=%d (segment0: policy에 따라 보통 pure dedup, 이후 segment는 online dev)\n",
            used_bytes, block_bytes, num_blocks_total,
            segment_idx + 1);

    return 0;
}

/* ============================================================
 * Decompression
 *   - 한 번에 하나의 segment 파일만 복원
 *   - segment마다 dev_pos_count, dev_len_per_block가 다를 수 있음
 *     (segment0는 dev_pos_count=0, dev_len=0)
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

    size_t block_bytes = (size_t)block_bytes_u32;
    int num_fields = (int)num_fields_u32;
    size_t dict_size = (size_t)dict_size_u32;
    size_t num_blocks = (size_t)num_blocks_u32;
    int dev_pos_count = (int)dev_pos_count_u32;     /* #bits */
    size_t dev_len_per_block = (size_t)dev_len_u32; /* bytes */

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
            /* dev 없음 (segment0) */
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
