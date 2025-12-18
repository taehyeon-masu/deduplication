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
 *      * output.ddp       : segment 0  (training: 완전 dedup, dict<=15)
 *      * output.ddp.seg1  : segment 1  (dev 포함, dict<=255)
 *      * output.ddp.seg2  : segment 2  (dev 포함, dict<=255)
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
    *out = (uint32_t)b[0] |
           ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) |
           ((uint32_t)b[3] << 24);
    return 1;
}

/* ============================================================
 * Bit helpers (block / deviation bitstream)
 * ============================================================ */

static inline uint8_t get_block_bit(const unsigned char *buf, size_t bit_pos)
{
    size_t byte_idx = bit_pos / 8;
    int    bit_idx  = (int)(bit_pos % 8); /* LSB-first */
    return (uint8_t)((buf[byte_idx] >> bit_idx) & 1u);
}

static inline void set_block_bit(unsigned char *buf, size_t bit_pos, uint8_t v)
{
    size_t byte_idx = bit_pos / 8;
    int    bit_idx  = (int)(bit_pos % 8);
    if (v)
        buf[byte_idx] |= (unsigned char)(1u << bit_idx);
    else
        buf[byte_idx] &= (unsigned char)~(1u << bit_idx);
}

static inline uint8_t get_dev_bit(const unsigned char *buf, size_t bit_idx)
{
    size_t byte_idx = bit_idx / 8;
    int    bit_in   = (int)(bit_idx % 8);
    return (uint8_t)((buf[byte_idx] >> bit_in) & 1u);
}

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
 * - dev_positions[] : block 내 비트 오프셋 (0 ~ block_bytes*8-1)
 * - dev_pos_count   : deviation bit 개수 (#bits)
 * - dev_len_per_block: deviation bitstream의 길이 (바이트)
 *      dev_len_per_block = ceil(dev_pos_count / 8)
 *
 *  비트 packing 규칙:
 *    - i번째 deviation bit (0-based)는
 *        dev_buf[ i/8 ] 의 (i%8)-th bit (LSB 기준)에 저장
 * ============================================================ */

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
 *
 *   - dev_pos_count == 0 이면:
 *       * dev_positions는 NULL 가능
 *       * dev_len_per_block == 0
 *       * deviation stream 없음 → "완전 dedup segment (training)"
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

/* ============================================================
 * multi-layout 압축 (DDP1)
 *
 * 전략:
 *  - segment 0 (training):
 *      * 완전 dedup (dev 없음, dev_pos_count=0)
 *      * dict size 최대 15개 (4bit ID 범위)
 *      * BitStats로 bit 변동 횟수 학습
 *
 *  - segment 1 부터:
 *      * bitstats 기반 top-N dev bit 선택
 *      * 선택된 dev bit는 dev stream으로 빼고, base block은 해당 비트를 0으로 한 상태로 dict에 저장
 *      * dict size 최대 255개 (1byte ID)
 *
 * 포맷 (각 segment 파일마다):
 *  magic: 'D','D','P','1'
 *  u32: block_bytes        (한 블록의 총 바이트 수, 원본)
 *  u32: num_fields         (블록 내부 field 개수)
 *  u32: dict_size          (사전에 저장된 base 블록 수)
 *  u32: num_blocks         (이 segment 안의 블록 수)
 *  u32: dev_pos_count      (# deviation bit positions, training seg에서는 0)
 *  u32: dev_len_per_block  (deviation bitstream 길이, bytes, training seg에서는 0)
 *  u32[num_fields]:  field_sizes
 *  u32[dev_pos_count]: dev_positions (segment 0 에서는 없음)
 *
 *  [dictionary]: dict_size * block_bytes bytes
 *  [block_ids]:  num_blocks * 1 byte (uint8_t)
 *  [deviation]:  num_blocks * dev_len_per_block bytes (bit-packed, segment 0에는 없음)
 * ============================================================ */

int compress_file(const char *input_filename,
                  const char *output_filename,
                  int num_fields,
                  const int *field_sizes,
                  int dev_top_n)
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

    if (dev_top_n < 0)
    {
        fprintf(stderr, "compress_file_multi: invalid dev_top_n (%d)\n", dev_top_n);
        return 1;
    }

    size_t block_bits = block_bytes * 8;

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

    /* BitStats: training segment 에서만 사용 (block_bits 기준) */
    BitStats *bs = bitstats_create(block_bits);
    if (!bs)
    {
        fprintf(stderr, "Failed to create BitStats\n");
        fclose(fin);
        return 1;
    }

    /* Training segment용 dictionary (완전 dedup, dict<=15) */
    const int TRAIN_DICT_LIMIT = 15;
    Dictionary dict_train;
    dict_init(&dict_train, block_bytes);

    /* Training segment용 block id 버퍼 (동적 확장) */
    size_t train_ids_cap   = 1024;
    size_t train_ids_count = 0;
    uint8_t *train_block_ids = (uint8_t *)malloc(train_ids_cap);
    if (!train_block_ids)
    {
        fprintf(stderr, "Failed to allocate train_block_ids\n");
        bitstats_free(bs);
        dict_free(&dict_train);
        fclose(fin);
        return 1;
    }

    /* Dev segment용 구조체: 초기에는 미사용 */
    const int DEV_DICT_LIMIT = 255;
    Dictionary dict_dev;
    int dict_dev_initialized = 0;

    uint8_t *dev_block_ids   = NULL;
    size_t   dev_ids_cap     = 0;
    size_t   dev_ids_count   = 0;

    unsigned char *dev_stream    = NULL;
    size_t         dev_stream_cap = 0;

    /* deviation bit 위치 (training 후에 결정) */
    int   *dev_positions    = NULL;
    int    dev_pos_count    = 0;
    size_t dev_len_per_block = 0;

    /* segment index: 0은 training, 1부터 dev segment */
    int segment_idx = 0;

    /* 상태 플래그 */
    int training_done = 0;

    /* 공용 block 버퍼 */
    unsigned char *block_buf = (unsigned char *)malloc(block_bytes);
    unsigned char *base_buf  = (unsigned char *)malloc(block_bytes); /* dev segment에서 base용 */
    unsigned char *dev_buf   = NULL; /* dev segment에서 dev bitstream 1block용 */

    if (!block_buf || !base_buf)
    {
        fprintf(stderr, "Failed to allocate block buffers\n");
        free(block_buf);
        free(base_buf);
        free(train_block_ids);
        bitstats_free(bs);
        dict_free(&dict_train);
        fclose(fin);
        return 1;
    }

    /* 메인 loop: 입력 파일 block-by-block 처리 */
    size_t global_block_idx = 0;

    while (global_block_idx < num_blocks_total)
    {
        size_t n = fread(block_buf, 1, block_bytes, fin);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read block %zu\n", global_block_idx);
            free(block_buf);
            free(base_buf);
            free(train_block_ids);
            free(dev_block_ids);
            free(dev_stream);
            free(dev_positions);
            if (dict_dev_initialized) dict_free(&dict_dev);
            bitstats_free(bs);
            dict_free(&dict_train);
            fclose(fin);
            return 1;
        }

        /* =========================
         * 1) Training segment
         *    - 완전 dedup
         *    - dict size <= 15
         *    - BitStats 업데이트
         * ========================= */
        if (!training_done)
        {
            /* BitStats에 현재 block 반영 */
            bitstats_update_block(bs, block_buf, block_bytes);

            int idx = dict_find(&dict_train, block_buf);

            if (idx == -1)
            {
                /* dict가 꽉 찬 상태에서 새로운 block을 만나면, training segment 종료 */
                if (dict_train.size >= TRAIN_DICT_LIMIT)
                {
                    /* 1-1) 지금까지의 training segment flush */
                    if (write_ddp1_segment(output_filename,
                                           segment_idx,          /* 0 */
                                           block_bytes,
                                           num_fields,
                                           field_sizes,
                                           0,                    /* dev_pos_count=0 */
                                           NULL,                 /* dev_positions 없음 */
                                           0,                    /* dev_len_per_block=0 */
                                           &dict_train,
                                           train_block_ids,
                                           NULL,                 /* dev_stream 없음 */
                                           train_ids_count) != 0)
                    {
                        fprintf(stderr, "Failed to write training segment\n");
                        free(block_buf);
                        free(base_buf);
                        free(train_block_ids);
                        free(dev_block_ids);
                        free(dev_stream);
                        free(dev_positions);
                        if (dict_dev_initialized) dict_free(&dict_dev);
                        bitstats_free(bs);
                        dict_free(&dict_train);
                        fclose(fin);
                        return 1;
                    }

                    fprintf(stderr, "[training] blocks=%zu, dict_size=%d\n",
                            train_ids_count, dict_train.size);

                    /* 1-2) BitStats 기반으로 dev bit 위치 선택 */
                    int max_dev_bits = dev_top_n;
                    if (max_dev_bits <= 0 || max_dev_bits > (int)block_bits)
                        max_dev_bits = (int)block_bits;

                    dev_positions = (int *)malloc(sizeof(int) * (size_t)max_dev_bits);
                    if (!dev_positions)
                    {
                        fprintf(stderr, "Failed to allocate dev_positions\n");
                        free(block_buf);
                        free(base_buf);
                        free(train_block_ids);
                        free(dev_block_ids);
                        free(dev_stream);
                        bitstats_free(bs);
                        dict_free(&dict_train);
                        fclose(fin);
                        return 1;
                    }

                    /* 정렬된 통계도 한번 출력해보기 (디버깅용) */
                    fprintf(stderr, "[training] bit change stats (sorted):\n");
                    bitstats_print_sorted(stderr, bs, block_bits);

                    size_t selected = bitstats_select_top_n(
                        bs,
                        (size_t)max_dev_bits,
                        dev_positions,
                        (size_t)max_dev_bits);

                    dev_pos_count = (int)selected;
                    dev_len_per_block = compute_dev_len_from_positions(dev_pos_count);

                    fprintf(stderr,
                            "[training] selected top %d bits for deviation "
                            "(dev_len_per_block=%zu bytes)\n",
                            dev_pos_count, dev_len_per_block);

                    /* Training용 구조체들 정리 */
                    bitstats_free(bs);
                    bs = NULL;
                    dict_free(&dict_train);
                    free(train_block_ids);
                    train_block_ids = NULL;
                    train_ids_cap = 0;
                    train_ids_count = 0;

                    /* 이제부터 dev segment 시작 */
                    training_done = 1;
                    segment_idx   = 1; /* 다음 segment index */

                    /* dev segment용 dictionary / buffer 초기화 */
                    if (dev_pos_count > 0)
                    {
                        dev_buf = (unsigned char *)malloc(dev_len_per_block);
                        if (!dev_buf)
                        {
                            fprintf(stderr, "Failed to allocate dev_buf\n");
                            free(block_buf);
                            free(base_buf);
                            free(dev_positions);
                            fclose(fin);
                            return 1;
                        }
                    }

                    dict_init(&dict_dev, block_bytes);
                    dict_dev_initialized = 1;

                    dev_ids_cap   = 1024;
                    dev_ids_count = 0;
                    dev_block_ids = (uint8_t *)malloc(dev_ids_cap);
                    if (!dev_block_ids)
                    {
                        fprintf(stderr, "Failed to allocate dev_block_ids\n");
                        free(block_buf);
                        free(base_buf);
                        free(dev_positions);
                        free(dev_buf);
                        dict_free(&dict_dev);
                        fclose(fin);
                        return 1;
                    }

                    if (dev_len_per_block > 0)
                    {
                        dev_stream_cap = dev_len_per_block * 1024;
                        dev_stream = (unsigned char *)malloc(dev_stream_cap);
                        if (!dev_stream)
                        {
                            fprintf(stderr, "Failed to allocate dev_stream\n");
                            free(block_buf);
                            free(base_buf);
                            free(dev_block_ids);
                            free(dev_positions);
                            free(dev_buf);
                            dict_free(&dict_dev);
                            fclose(fin);
                            return 1;
                        }
                    }

                    /* 현재 block(새로운 block)은 dev segment 규칙으로 처리해야 하므로
                     * dev segment 경로로 바로 넘어감 */
                    goto DEV_SEGMENT_PROCESS;
                }
                else
                {
                    idx = dict_add(&dict_train, block_buf);
                    if (idx < 0)
                    {
                        fprintf(stderr, "dict_add (training) failed\n");
                        free(block_buf);
                        free(base_buf);
                        free(train_block_ids);
                        bitstats_free(bs);
                        dict_free(&dict_train);
                        fclose(fin);
                        return 1;
                    }
                }
            }

            /* training segment용 block id push */
            if (train_ids_count >= train_ids_cap)
            {
                size_t new_cap = train_ids_cap * 2;
                uint8_t *tmp = (uint8_t *)realloc(train_block_ids, new_cap);
                if (!tmp)
                {
                    fprintf(stderr, "realloc train_block_ids failed\n");
                    free(block_buf);
                    free(base_buf);
                    free(train_block_ids);
                    bitstats_free(bs);
                    dict_free(&dict_train);
                    fclose(fin);
                    return 1;
                }
                train_block_ids = tmp;
                train_ids_cap   = new_cap;
            }
            train_block_ids[train_ids_count++] = (uint8_t)idx;
        }
        else
        {
            /* =========================
             * 2) Dev segment (training 끝난 후)
             * ========================= */
DEV_SEGMENT_PROCESS: ;

            if (!dict_dev_initialized)
            {
                /* (이론상 여기 들어오면 안 되지만 방어용) */
                dict_init(&dict_dev, block_bytes);
                dict_dev_initialized = 1;

                dev_ids_cap   = 1024;
                dev_ids_count = 0;
                dev_block_ids = (uint8_t *)malloc(dev_ids_cap);

                if (!dev_block_ids)
                {
                    fprintf(stderr, "Failed to allocate dev_block_ids\n");
                    free(block_buf);
                    free(base_buf);
                    free(dev_positions);
                    fclose(fin);
                    return 1;
                }

                if (dev_pos_count > 0)
                {
                    dev_len_per_block = compute_dev_len_from_positions(dev_pos_count);
                    dev_buf = (unsigned char *)malloc(dev_len_per_block);
                    if (!dev_buf)
                    {
                        fprintf(stderr, "Failed to allocate dev_buf\n");
                        free(block_buf);
                        free(base_buf);
                        free(dev_block_ids);
                        free(dev_positions);
                        dict_free(&dict_dev);
                        fclose(fin);
                        return 1;
                    }

                    dev_stream_cap = dev_len_per_block * 1024;
                    dev_stream = (unsigned char *)malloc(dev_stream_cap);
                    if (!dev_stream)
                    {
                        fprintf(stderr, "Failed to allocate dev_stream\n");
                        free(block_buf);
                        free(base_buf);
                        free(dev_block_ids);
                        free(dev_positions);
                        free(dev_buf);
                        dict_free(&dict_dev);
                        fclose(fin);
                        return 1;
                    }
                }
            }

            /* block_buf를 base/dev로 분리하기 전에 복사본 유지 (필요시) */
            memcpy(base_buf, block_buf, block_bytes);

            /* dev bit 추출 & base bit 0으로 만들기 */
            if (dev_pos_count > 0 && dev_len_per_block > 0)
            {
                size_t used_dev = extract_base_and_deviation_by_pos(
                    base_buf,          /* base용 버퍼 */
                    block_bytes,
                    dev_positions,
                    dev_pos_count,
                    dev_buf,
                    dev_len_per_block);

                if (used_dev != dev_len_per_block)
                {
                    fprintf(stderr,
                            "extract_base_and_deviation_by_pos: used_dev=%zu, expected=%zu\n",
                            used_dev, dev_len_per_block);
                    free(block_buf);
                    free(base_buf);
                    free(train_block_ids);
                    free(dev_block_ids);
                    free(dev_stream);
                    free(dev_positions);
                    free(dev_buf);
                    dict_free(&dict_dev);
                    fclose(fin);
                    return 1;
                }
            }
            else
            {
                /* dev_pos_count == 0 인 경우: dev 없음, base는 원본 */
                memcpy(base_buf, block_buf, block_bytes);
            }

            /* dev segment의 dictionary lookup/insert (base 기준) */
            int idx = dict_find(&dict_dev, base_buf);
            if (idx == -1)
            {
                if (dict_dev.size >= DEV_DICT_LIMIT)
                {
                    /* 현재 segment flush */
                    if (write_ddp1_segment(output_filename,
                                           segment_idx,
                                           block_bytes,
                                           num_fields,
                                           field_sizes,
                                           dev_pos_count,
                                           dev_positions,
                                           dev_len_per_block,
                                           &dict_dev,
                                           dev_block_ids,
                                           dev_stream,
                                           dev_ids_count) != 0)
                    {
                        fprintf(stderr, "Failed to write dev segment %d\n",
                                segment_idx);
                        free(block_buf);
                        free(base_buf);
                        free(train_block_ids);
                        free(dev_block_ids);
                        free(dev_stream);
                        free(dev_positions);
                        free(dev_buf);
                        dict_free(&dict_dev);
                        fclose(fin);
                        return 1;
                    }

                    dict_free(&dict_dev);
                    dict_init(&dict_dev, block_bytes);

                    dev_ids_count = 0;
                    /* dev_block_ids, dev_stream 버퍼는 재사용 (cap 유지) */

                    segment_idx++;
                }

                idx = dict_add(&dict_dev, base_buf);
                if (idx < 0)
                {
                    fprintf(stderr, "dict_add (dev) failed\n");
                    free(block_buf);
                    free(base_buf);
                    free(train_block_ids);
                    free(dev_block_ids);
                    free(dev_stream);
                    free(dev_positions);
                    free(dev_buf);
                    dict_free(&dict_dev);
                    fclose(fin);
                    return 1;
                }
            }

            /* dev segment block id push */
            if (dev_ids_count >= dev_ids_cap)
            {
                size_t new_cap = dev_ids_cap * 2;
                uint8_t *tmp = (uint8_t *)realloc(dev_block_ids, new_cap);
                if (!tmp)
                {
                    fprintf(stderr, "realloc dev_block_ids failed\n");
                    free(block_buf);
                    free(base_buf);
                    free(train_block_ids);
                    free(dev_block_ids);
                    free(dev_stream);
                    free(dev_positions);
                    free(dev_buf);
                    dict_free(&dict_dev);
                    fclose(fin);
                    return 1;
                }
                dev_block_ids = tmp;
                dev_ids_cap   = new_cap;
            }
            dev_block_ids[dev_ids_count] = (uint8_t)idx;

            /* dev stream에 dev_buf append */
            if (dev_len_per_block > 0 && dev_pos_count > 0)
            {
                size_t needed = (dev_ids_count + 1) * dev_len_per_block;
                if (needed > dev_stream_cap)
                {
                    size_t new_cap = dev_stream_cap * 2;
                    if (new_cap < needed)
                        new_cap = needed;

                    unsigned char *tmp = (unsigned char *)realloc(dev_stream, new_cap);
                    if (!tmp)
                    {
                        fprintf(stderr, "realloc dev_stream failed\n");
                        free(block_buf);
                        free(base_buf);
                        free(train_block_ids);
                        free(dev_block_ids);
                        free(dev_stream);
                        free(dev_positions);
                        free(dev_buf);
                        dict_free(&dict_dev);
                        fclose(fin);
                        return 1;
                    }
                    dev_stream     = tmp;
                    dev_stream_cap = new_cap;
                }

                memcpy(dev_stream + dev_ids_count * dev_len_per_block,
                       dev_buf,
                       dev_len_per_block);
            }

            dev_ids_count++;
        }

        global_block_idx++;
    }

    fclose(fin);

    /* ====== EOF 처리 ====== */

    int ret = 0;

    if (!training_done)
    {
        /* 파일 전체가 training segment로만 끝난 경우 (dict <= 15, dev 없음) */
        if (write_ddp1_segment(output_filename,
                               0,
                               block_bytes,
                               num_fields,
                               field_sizes,
                               0,
                               NULL,
                               0,
                               &dict_train,
                               train_block_ids,
                               NULL,
                               train_ids_count) != 0)
        {
            fprintf(stderr, "Failed to write final training-only segment\n");
            ret = 1;
        }
    }
    else
    {
        /* dev segment가 남아 있으면 flush */
        if (dev_ids_count > 0)
        {
            if (write_ddp1_segment(output_filename,
                                   segment_idx,
                                   block_bytes,
                                   num_fields,
                                   field_sizes,
                                   dev_pos_count,
                                   dev_positions,
                                   dev_len_per_block,
                                   &dict_dev,
                                   dev_block_ids,
                                   dev_stream,
                                   dev_ids_count) != 0)
            {
                fprintf(stderr, "Failed to write final dev segment %d\n",
                        segment_idx);
                ret = 1;
            }
        }
    }

    /* 정리 */
    free(block_buf);
    free(base_buf);
    free(train_block_ids);
    free(dev_block_ids);
    free(dev_stream);
    free(dev_positions);
    free(dev_buf);

    if (bs)            bitstats_free(bs);
    dict_free(&dict_train);
    if (dict_dev_initialized) dict_free(&dict_dev);

    fprintf(stderr,
            "Compressed (DDP1 multi, segmented, training+dev): "
            "used_bytes=%zu, block_bytes=%zu, total_blocks=%zu\n",
            used_bytes, block_bytes, num_blocks_total);

    return ret;
}

/* ============================================================
 * Decompression
 *   - segment 단위로 호출 (output.ddp, output.ddp.seg1, ...)
 *   - training segment(dev_pos_count=0) / dev segment 둘 다 처리 가능
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
