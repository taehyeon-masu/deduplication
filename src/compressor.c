#include "../include/compressor.h"
#include "../include/dictionary.h"
#include "../include/bin_io.h"

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
    *out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return 1;
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
 * Deviation: 위치 기반 설정
 *
 * - 압축 시: dev_positions[] 에 적힌 block 내 오프셋에서만 바이트를 뽑아 dev_buf에 저장
 * - dev_len_per_block = (#positions)  (padding 안 함)
 *
 *  예) dev_positions 0,2,4   → dev_len_per_block = 3
 *      dev_positions 0,2,4,8 → dev_len_per_block = 4
 *
 *  디코딩 시: 헤더에 저장된 dev_positions[]를 그대로 사용
 * ============================================================ */

/* (압축기에서 기본 deviation 위치 설정)
 *   => 이 배열과 개수를 바꾸면 deviation 패턴이 바뀜
 *   => block_bytes 보다 작은 오프셋만 사용해야 함
 *
 *   예: block_bytes = 16 (T(2),RH(2),lux1(2),P1(4),lux2(2),P2(4))
 *       아래 예시는 P1, P2의 하위 2바이트만 deviation으로:
 *          T    : 0,1
 *          RH   : 2,3
 *          lux1 : 4,5
 *          P1   : 6,7,8,9
 *          lux2 : 10,11
 *          P2   : 12,13,14,15
 *       g_dev_positions = { 6, 7, 12, 13 } 와 같이 설정 가능
 */
static const int g_dev_positions[] = {
    /* 예시: P1 하위 2바이트(6,7), P2 하위 2바이트(12,13) */
    4, 6, 10, 13};

static const int g_num_dev_positions =
    (int)(sizeof(g_dev_positions) / sizeof(g_dev_positions[0]));

static size_t compute_dev_len_from_positions(int num_pos)
{
    if (num_pos < 0)
        return 0;
    return (size_t)num_pos; /* padding 없이 그대로 사용 */
}

/* block_buf에서 선택된 바이트들을 deviation 버퍼(dev_buf)에 모으고,
 * block_buf의 해당 위치는 0으로 만들어 base만 남긴다.
 *
 * dev_buf: 최소 dev_len_per_block 만큼 할당되어 있어야 함.
 * dev_buf[0..num_dev_pos-1]   : 실제 deviation 값
 * dev_buf[num_dev_pos..end-1] : 0 (현재는 raw == dev_len_per_block 이므로 의미는 거의 없음)
 *
 * 반환값: dev_len_per_block (정상일 때)
 */
static size_t extract_base_and_deviation_by_pos(
    unsigned char *block_buf,
    size_t block_bytes,
    const int *dev_positions,
    int num_dev_pos,
    unsigned char *dev_buf,
    size_t dev_len_per_block)
{
    if (!block_buf || !dev_positions || !dev_buf || num_dev_pos < 0)
        return 0;

    if ((size_t)num_dev_pos > dev_len_per_block)
    {
        fprintf(stderr,
                "extract_base_and_deviation_by_pos: num_dev_pos(%d) > dev_len_per_block(%zu)\n",
                num_dev_pos, dev_len_per_block);
        return 0;
    }

    /* 1) deviation 데이터 채우기 */
    for (int i = 0; i < num_dev_pos; ++i)
    {
        int pos = dev_positions[i];
        if (pos < 0 || (size_t)pos >= block_bytes)
        {
            fprintf(stderr,
                    "extract_base_and_deviation_by_pos: invalid pos=%d (block_bytes=%zu)\n",
                    pos, block_bytes);
            return 0;
        }
        dev_buf[i] = block_buf[pos];
        block_buf[pos] = 0; /* base에서는 해당 위치를 0으로 */
    }

    /* 2) 나머지 padding 영역은 0으로 초기화 (필수는 아니지만 안전) */
    for (size_t i = (size_t)num_dev_pos; i < dev_len_per_block; ++i)
    {
        dev_buf[i] = 0;
    }

    return dev_len_per_block;
}

/* base_block + dev_buf → out_block (byte 위치 기반)
 *
 *  - base_block: dict에 저장된 base 블록 (dev 위치는 0이어야 함)
 *  - dev_buf   : dev_len_per_block 크기 버퍼, 앞 num_dev_pos 바이트만 의미 있음
 *
 * 반환값: 실제 deviation 바이트 개수(raw) — 보통 num_dev_pos 와 같음
 */
static size_t merge_base_and_deviation_by_pos(
    const unsigned char *base_block,
    size_t block_bytes,
    const int *dev_positions,
    int num_dev_pos,
    const unsigned char *dev_buf,
    size_t dev_len_per_block,
    unsigned char *out_block)
{
    if (!base_block || !dev_positions || !dev_buf || !out_block)
        return 0;

    /* 우선 out_block = base_block */
    memcpy(out_block, base_block, block_bytes);

    if ((size_t)num_dev_pos > dev_len_per_block)
    {
        fprintf(stderr,
                "merge_base_and_deviation_by_pos: num_dev_pos(%d) > dev_len_per_block(%zu)\n",
                num_dev_pos, dev_len_per_block);
        return 0;
    }

    size_t raw = (size_t)num_dev_pos;

    /* dev_positions에 해당하는 위치에 dev_buf의 앞 raw 바이트만 적용 */
    for (int i = 0; i < num_dev_pos; ++i)
    {
        int pos = dev_positions[i];
        if (pos < 0 || (size_t)pos >= block_bytes)
        {
            fprintf(stderr,
                    "merge_base_and_deviation_by_pos: invalid pos=%d (block_bytes=%zu)\n",
                    pos, block_bytes);
            return 0;
        }
        out_block[pos] = dev_buf[i];
    }

    (void)dev_len_per_block; /* 필요시 추가 검증용으로 사용 가능 */
    return raw;
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

    uint32_t block_bytes_u32 = (uint32_t)block_bytes;
    uint32_t num_fields_u32 = (uint32_t)num_fields;
    uint32_t dict_size_u32 = (uint32_t)dict->size;
    uint32_t num_blocks_u32 = (uint32_t)num_blocks_segment;
    uint32_t dev_pos_count_u32 = (uint32_t)dev_pos_count;
    uint32_t dev_len_u32 = (uint32_t)dev_len_per_block;

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

    /* dev_positions[] */
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

    /* deviation stream */
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
            "[segment %d] wrote '%s': blocks=%zu, dict_size=%d, dev_len_per_block=%zu\n",
            segment_idx, filename, num_blocks_segment, dict->size, dev_len_per_block);

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
 *  u32: dev_pos_count      (# deviation positions)
 *  u32: dev_len_per_block  (deviation 길이)
 *  u32[num_fields]:  field_sizes  (각 field의 byte 수)
 *  u32[dev_pos_count]: dev_positions (block 내 deviation byte 오프셋들)
 *
 *  [dictionary]: dict_size * block_bytes bytes
 *  [block_ids]:  num_blocks * 1 byte (uint8_t, 0 ~ 255)
 *  [deviation]:  num_blocks * dev_len_per_block bytes
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

    /* deviation 길이 계산 (전역 g_dev_positions 사용) */
    if (g_num_dev_positions < 0)
    {
        fprintf(stderr, "compress_file_multi: invalid g_num_dev_positions\n");
        return 1;
    }
    size_t dev_len_per_block = compute_dev_len_from_positions(g_num_dev_positions);

    /* dev_positions가 block_bytes 범위 안에 있는지 확인 */
    for (int i = 0; i < g_num_dev_positions; ++i)
    {
        int pos = g_dev_positions[i];
        if (pos < 0 || (size_t)pos >= block_bytes)
        {
            fprintf(stderr,
                    "compress_file_multi: dev position %d out of range (block_bytes=%zu)\n",
                    pos, block_bytes);
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

    /* segment 관리 변수 */
    int segment_idx = 0;            /* 0 -> output.ddp, 1 -> output.ddp.seg1 ... */
    size_t global_b = 0;            /* 전체 block index */
    size_t segment_start_block = 0; /* 이 segment가 시작되는 global block index */
    size_t segment_block_count = 0; /* 이 segment 안에 실제로 포함된 block 수 */

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
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        /* deviation 추출 */
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
         */
        int idx = dict_find(&dict, block_buf);
        if (idx == -1)
        {
            if (dict.size >= 255)
            {
                /* 현재 segment flush */
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
                        dict_free(&dict);
                        fclose(fin);
                        return 1;
                    }
                }

                /* 새 segment 시작 */
                dict_free(&dict);
                dict_init(&dict, block_bytes);

                segment_idx++;
                segment_start_block = global_b;
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
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        block_ids[global_b] = (uint8_t)idx;
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
            dict_free(&dict);
            return 1;
        }
    }

    free(block_buf);
    free(dev_buf);
    free(dev_stream);
    free(block_ids);
    dict_free(&dict);

    fprintf(stderr,
            "Compressed (DDP1 multi, segmented): used_bytes=%zu, "
            "block_bytes=%zu, total_blocks=%zu, dev_len_per_block=%zu, segments=%d\n",
            used_bytes, block_bytes, num_blocks_total,
            dev_len_per_block, segment_idx + 1);

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

    size_t block_bytes = (size_t)block_bytes_u32;
    int num_fields = (int)num_fields_u32;
    size_t dict_size = (size_t)dict_size_u32;
    size_t num_blocks = (size_t)num_blocks_u32;
    int dev_pos_count = (int)dev_pos_count_u32;
    size_t dev_len_per_block = (size_t)dev_len_u32;

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

    /* deviation positions 읽기 */
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
            if (dev_positions[i] < 0 || (size_t)dev_positions[i] >= block_bytes)
            {
                fprintf(stderr,
                        "dev_positions[%d]=%d out of range (block_bytes=%zu)\n",
                        i, dev_positions[i], block_bytes);
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

    /* deviation stream 읽기 */
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
