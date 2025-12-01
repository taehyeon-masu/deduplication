#include "../include/compressor.h"
#include "../include/dictionary.h"
#include "../include/bin_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

size_t extract_base_and_deviation(
    unsigned char *block_buf,
    int width_bytes,
    int block_size_samples,
    unsigned char *dev_buf);

size_t merge_base_and_deviation(
    const unsigned char *base_block,
    int width_bytes,
    int block_size_samples,
    const unsigned char *dev_buf,
    unsigned char *out_block);

static int write_u32_le(FILE *fp, uint32_t v)
{
    unsigned char b[4];
    b[0] = (unsigned char)(v & 0xFF);
    b[1] = (unsigned char)((v >> 8) & 0xFF);
    b[2] = (unsigned char)((v >> 16) & 0xFF);
    b[3] = (unsigned char)((v >> 24) & 0xFF);
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

// dev_len 은 width_bytes, block_size_samples 에 의해 완전히 결정되므로
// 헤더에 따로 저장하지 않고 양쪽에서 동일한 공식을 사용한다.
static size_t compute_dev_len_per_block(int width_bytes, int block_size_samples)
{
    int base_low_bytes = width_bytes / 2;
    int has_extra_byte = (width_bytes % 2); // 홀수면 마지막 sample에서 1 byte 추가
    return (size_t)block_size_samples * (size_t)base_low_bytes +
           (has_extra_byte ? 1u : 0u);
}

// 포맷:
//  magic: 'D','D','P','1' (4바이트)
//  u32: sample_count (압축에 사용된 샘플 수 = used_samples)
//  u32: block_size_samples
//  u8 : width_bytes
//  u8[3]: reserved(0)
//  u32: dict_size
//  u32: num_blocks
//  [dictionary]: dict_size * (block_size_samples * width_bytes) bytes  (base 블록들)
//  [block_ids]:  num_blocks * 4 bytes (u32, LE)
//  [deviation_stream]:
//      num_blocks * dev_len_per_block bytes
//      (각 블록의 deviation들이 순서대로, 고정 길이 dev_len_per_block)

int compress_file(const char *input_filename,
                  const char *output_filename,
                  int width_bytes,
                  int block_size_samples)
{
    if (!(width_bytes == 1 || width_bytes == 2 ||
          width_bytes == 4 || width_bytes == 8))
    {
        fprintf(stderr, "width_bytes must be 1,2,4,8\n");
        return 1;
    }
    if (block_size_samples <= 0)
    {
        fprintf(stderr, "block_size_samples must be positive\n");
        return 1;
    }

    // 1) 입력 파일 열기
    FILE *fin = fopen(input_filename, "rb");
    if (!fin)
    {
        perror("fopen input");
        return 1;
    }

    // 2) 파일 크기 구하기 (전체를 읽지 않고 ftell 사용)
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
    if (nbytes < (size_t)width_bytes)
    {
        fprintf(stderr, "Input file too small\n");
        fclose(fin);
        return 1;
    }

    size_t total_samples = nbytes / (size_t)width_bytes;
    if (total_samples == 0)
    {
        fprintf(stderr, "No full samples found\n");
        fclose(fin);
        return 1;
    }

    size_t num_blocks = total_samples / (size_t)block_size_samples;
    if (num_blocks == 0)
    {
        fprintf(stderr,
                "Not enough samples for at least one full block "
                "(need >= %d samples)\n",
                block_size_samples);
        fclose(fin);
        return 1;
    }

    size_t used_samples = num_blocks * (size_t)block_size_samples;
    size_t block_size_bytes = (size_t)block_size_samples * (size_t)width_bytes;

    if (used_samples < total_samples)
    {
        fprintf(stderr,
                "Warning: last %zu samples (%zu bytes) are ignored (not enough to fill a block)\n",
                (total_samples - used_samples),
                (total_samples - used_samples) * (size_t)width_bytes);
    }

    // 3) block_ids 배열만 메모리에 보관 (원본 데이터 전체는 안 올림)
    uint32_t *block_ids = (uint32_t *)malloc(sizeof(uint32_t) * num_blocks);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        fclose(fin);
        return 1;
    }

    // 4) dictionary 초기화 + 블록 버퍼 1개만 사용
    Dictionary dict;
    dict_init(&dict, block_size_bytes);

    unsigned char *block_buf = (unsigned char *)malloc(block_size_bytes);
    if (!block_buf)
    {
        fprintf(stderr, "Failed to allocate block buffer\n");
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    // deviation 버퍼는 "한 블록의 deviation 크기"만큼만 할당
    size_t dev_len_per_block = compute_dev_len_per_block(width_bytes, block_size_samples);
    unsigned char *dev_buf = (unsigned char *)malloc(dev_len_per_block);
    if (!dev_buf)
    {
        fprintf(stderr, "Failed to allocate deviation buffer\n");
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    // 5) 1-pass: 파일을 블록 단위로 순차적으로 읽으면서
    //    base 블록으로 dict 구성 + block_ids 기록
    for (size_t b = 0; b < num_blocks; ++b)
    {
        size_t n = fread(block_buf, 1, block_size_bytes, fin);
        if (n != block_size_bytes)
        {
            fprintf(stderr, "Failed to read block %zu from input\n", b);
            free(dev_buf);
            free(block_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        size_t cur_dev_len = extract_base_and_deviation(
            block_buf,
            width_bytes,
            block_size_samples,
            dev_buf);

        if (cur_dev_len != dev_len_per_block)
        {
            fprintf(stderr,
                    "Inconsistent deviation length: got %zu, expected %zu\n",
                    cur_dev_len, dev_len_per_block);
            free(dev_buf);
            free(block_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        int idx = dict_find(&dict, block_buf);
        if (idx == -1)
        {
            idx = dict_add(&dict, block_buf);
        }
        block_ids[b] = (uint32_t)idx;
    }

    // fin 은 deviation 스트림 쓰기 위해 2-pass 용으로 다시 사용할 예정

    // 6) 헤더 + dictionary + block_ids 쓰기

    FILE *fp = fopen(output_filename, "wb");
    if (!fp)
    {
        perror("fopen output");
        free(dev_buf);
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    const unsigned char magic[4] = {'D', 'D', 'P', '1'};
    if (fwrite(magic, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to write magic\n");
        fclose(fp);
        free(dev_buf);
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    uint32_t sample_count_u32 = (uint32_t)used_samples;
    uint32_t block_size_samples_u32 = (uint32_t)block_size_samples;
    uint32_t dict_size_u32 = (uint32_t)dict.size;
    uint32_t num_blocks_u32 = (uint32_t)num_blocks;

    if (!write_u32_le(fp, sample_count_u32) ||
        !write_u32_le(fp, block_size_samples_u32))
    {
        fprintf(stderr, "Failed to write header\n");
        fclose(fp);
        free(dev_buf);
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    unsigned char header_extra[4];
    header_extra[0] = (unsigned char)width_bytes;
    header_extra[1] = 0;
    header_extra[2] = 0;
    header_extra[3] = 0;
    if (fwrite(header_extra, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to write header extra\n");
        fclose(fp);
        free(dev_buf);
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    if (!write_u32_le(fp, dict_size_u32) ||
        !write_u32_le(fp, num_blocks_u32))
    {
        fprintf(stderr, "Failed to write header tail\n");
        fclose(fp);
        free(dev_buf);
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    // dictionary 블록 쓰기 (base만 들어 있음)
    for (int i = 0; i < dict.size; ++i)
    {
        if (fwrite(dict.blocks[i], 1, block_size_bytes, fp) != block_size_bytes)
        {
            fprintf(stderr, "Failed to write dictionary block %d\n", i);
            fclose(fp);
            free(dev_buf);
            free(block_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }
    }

    // block_ids 쓰기
    for (size_t b = 0; b < num_blocks; ++b)
    {
        if (!write_u32_le(fp, block_ids[b]))
        {
            fprintf(stderr, "Failed to write block id %zu\n", b);
            fclose(fp);
            free(dev_buf);
            free(block_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }
    }

    // 7) 2-pass: 입력 파일을 다시 처음부터 읽으면서,
    //    deviation만 dev_buf에 담아 그대로 output 스트림 끝에 append
    if (fseek(fin, 0, SEEK_SET) != 0)
    {
        perror("fseek set (second pass)");
        fclose(fp);
        free(dev_buf);
        free(block_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fin);
        return 1;
    }

    for (size_t b = 0; b < num_blocks; ++b)
    {
        size_t n = fread(block_buf, 1, block_size_bytes, fin);
        if (n != block_size_bytes)
        {
            fprintf(stderr, "Failed to read block %zu from input (2nd pass)\n", b);
            fclose(fp);
            free(dev_buf);
            free(block_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        size_t cur_dev_len = extract_base_and_deviation(
            block_buf,
            width_bytes,
            block_size_samples,
            dev_buf);
        if (cur_dev_len != dev_len_per_block)
        {
            fprintf(stderr,
                    "Inconsistent deviation length in 2nd pass: got %zu, expected %zu\n",
                    cur_dev_len, dev_len_per_block);
            fclose(fp);
            free(dev_buf);
            free(block_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        if (dev_len_per_block > 0)
        {
            if (fwrite(dev_buf, 1, dev_len_per_block, fp) != dev_len_per_block)
            {
                fprintf(stderr, "Failed to write deviation for block %zu\n", b);
                fclose(fp);
                free(dev_buf);
                free(block_buf);
                free(block_ids);
                dict_free(&dict);
                fclose(fin);
                return 1;
            }
        }
    }

    fclose(fin);
    fclose(fp);
    free(dev_buf);
    free(block_buf);
    free(block_ids);
    dict_free(&dict);

    fprintf(stderr,
            "Compressed: used_samples=%zu, block_size_samples=%d, dict_size=%d, num_blocks=%zu, dev_len_per_block=%zu\n",
            used_samples, block_size_samples, dict.size, num_blocks, dev_len_per_block);

    return 0;
}

// base 블록에서 하위 바이트들을 떼어내 deviation에 저장하고, 그 부분을 0으로 만든다.
size_t extract_base_and_deviation(
    unsigned char *block_buf,
    int width_bytes,
    int block_size_samples,
    unsigned char *dev_buf)
{
    size_t dev_offset = 0;

    if (width_bytes <= 0 || block_size_samples <= 0)
    {
        return 0;
    }

    int base_low_bytes = width_bytes / 2;   // 각 sample에서 기본적으로 떼어낼 byte 수
    int has_extra_byte = (width_bytes % 2); // 홀수면 마지막 sample에서 1 byte 추가

    for (int s = 0; s < block_size_samples; ++s)
    {
        unsigned char *sample = block_buf + s * width_bytes;

        int take = base_low_bytes;
        // 마지막 sample이고 width_bytes가 홀수라면 1 byte 더 가져감
        if (has_extra_byte && s == block_size_samples - 1)
        {
            take += 1;
        }

        if (take > 0)
        {
            // deviation에 하위 take bytes 복사
            memcpy(dev_buf + dev_offset, sample, (size_t)take);
            dev_offset += (size_t)take;

            // base에서는 이 부분을 0으로 만들어 deviation을 제거
            memset(sample, 0, (size_t)take);
        }
        // 나머지 상위 바이트들은 base로 그대로 둠
    }

    return dev_offset; // deviation 길이 (bytes)
}

// base 블록 + deviation 버퍼에서 원본 블록을 복원한다.
// (extract_base_and_deviation의 역연산)
size_t merge_base_and_deviation(
    const unsigned char *base_block,
    int width_bytes,
    int block_size_samples,
    const unsigned char *dev_buf,
    unsigned char *out_block)
{
    size_t dev_offset = 0;

    if (width_bytes <= 0 || block_size_samples <= 0)
    {
        return 0;
    }

    int base_low_bytes = width_bytes / 2;
    int has_extra_byte = (width_bytes % 2);

    for (int s = 0; s < block_size_samples; ++s)
    {
        const unsigned char *base_sample = base_block + s * width_bytes;
        unsigned char *out_sample = out_block + s * width_bytes;

        int take = base_low_bytes;
        if (has_extra_byte && s == block_size_samples - 1)
        {
            take += 1;
        }

        // 하위 take 바이트는 deviation에서 가져온다.
        if (take > 0)
        {
            memcpy(out_sample, dev_buf + dev_offset, (size_t)take);
            dev_offset += (size_t)take;
        }

        // 나머지 상위 바이트는 base_sample에서 가져온다.
        int remain = width_bytes - take;
        if (remain > 0)
        {
            memcpy(out_sample + take, base_sample + take, (size_t)remain);
        }
    }

    return dev_offset; // 사용한 deviation 길이 (bytes)
}

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
        fprintf(stderr, "Failed to read magic\n");
        fclose(fp);
        return 1;
    }
    if (!(magic[0] == 'D' && magic[1] == 'D' &&
          magic[2] == 'P' && magic[3] == '1'))
    {
        fprintf(stderr, "Invalid magic, not a DDP1 file\n");
        fclose(fp);
        return 1;
    }

    uint32_t sample_count_u32;
    uint32_t block_size_samples_u32;
    if (!read_u32_le(fp, &sample_count_u32) ||
        !read_u32_le(fp, &block_size_samples_u32))
    {
        fprintf(stderr, "Failed to read header\n");
        fclose(fp);
        return 1;
    }

    unsigned char header_extra[4];
    if (fread(header_extra, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to read header extra\n");
        fclose(fp);
        return 1;
    }
    int width_bytes = (int)header_extra[0];
    if (!(width_bytes == 1 || width_bytes == 2 ||
          width_bytes == 4 || width_bytes == 8))
    {
        fprintf(stderr, "Invalid width_bytes in header: %d\n", width_bytes);
        fclose(fp);
        return 1;
    }

    uint32_t dict_size_u32;
    uint32_t num_blocks_u32;
    if (!read_u32_le(fp, &dict_size_u32) ||
        !read_u32_le(fp, &num_blocks_u32))
    {
        fprintf(stderr, "Failed to read header tail\n");
        fclose(fp);
        return 1;
    }

    size_t sample_count = (size_t)sample_count_u32;
    size_t block_size_samples = (size_t)block_size_samples_u32;
    size_t dict_size = (size_t)dict_size_u32;
    size_t num_blocks = (size_t)num_blocks_u32;
    size_t block_size_bytes = block_size_samples * (size_t)width_bytes;

    // dev_len_per_block 은 compress와 동일한 공식으로 계산
    size_t dev_len_per_block = compute_dev_len_per_block(width_bytes, (int)block_size_samples);

    Dictionary dict;
    dict_init(&dict, block_size_bytes);

    // dictionary(base 블록들) 읽기
    for (size_t i = 0; i < dict_size; ++i)
    {
        unsigned char *buf = (unsigned char *)malloc(block_size_bytes);
        if (!buf)
        {
            fprintf(stderr, "Failed to allocate block buffer\n");
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        size_t n = fread(buf, 1, block_size_bytes, fp);
        if (n != block_size_bytes)
        {
            fprintf(stderr, "Failed to read dictionary block %zu\n", i);
            free(buf);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        dict_add(&dict, buf);
        free(buf);
    }

    // block_ids 읽기
    uint32_t *block_ids = (uint32_t *)malloc(sizeof(uint32_t) * num_blocks);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        dict_free(&dict);
        fclose(fp);
        return 1;
    }
    for (size_t b = 0; b < num_blocks; ++b)
    {
        if (!read_u32_le(fp, &block_ids[b]))
        {
            fprintf(stderr, "Failed to read block id %zu\n", b);
            free(block_ids);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
    }

    // 이제 fp 위치는 deviation_stream의 시작 지점
    unsigned char *dev_buf = NULL;
    if (dev_len_per_block > 0)
    {
        dev_buf = (unsigned char *)malloc(dev_len_per_block);
        if (!dev_buf)
        {
            fprintf(stderr, "Failed to allocate dev_buf\n");
            free(block_ids);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
    }

    size_t total_bytes = sample_count * (size_t)width_bytes;
    unsigned char *out = (unsigned char *)malloc(total_bytes);
    if (!out)
    {
        fprintf(stderr, "Failed to allocate output buffer\n");
        free(dev_buf);
        free(block_ids);
        dict_free(&dict);
        fclose(fp);
        return 1;
    }

    size_t bytes_written = 0;

    for (size_t b = 0; b < num_blocks; ++b)
    {
        uint32_t id = block_ids[b];
        if (id >= dict.size)
        {
            fprintf(stderr, "Invalid dictionary id %u at block %zu\n", id, b);
            free(out);
            free(dev_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        const unsigned char *base_block = dict.blocks[id];

        // deviation 읽기
        if (dev_len_per_block > 0)
        {
            size_t n = fread(dev_buf, 1, dev_len_per_block, fp);
            if (n != dev_len_per_block)
            {
                fprintf(stderr, "Failed to read deviation for block %zu\n", b);
                free(out);
                free(dev_buf);
                free(block_ids);
                dict_free(&dict);
                fclose(fp);
                return 1;
            }
        }

        // base + deviation 을 합쳐 원본 블록을 out에 복원
        unsigned char *out_block = out + bytes_written;
        size_t used_dev = merge_base_and_deviation(
            base_block,
            width_bytes,
            (int)block_size_samples,
            dev_buf,
            out_block);

        if (used_dev != dev_len_per_block)
        {
            fprintf(stderr, "merge used_dev=%zu != dev_len_per_block=%zu\n",
                    used_dev, dev_len_per_block);
            free(out);
            free(dev_buf);
            free(block_ids);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }

        bytes_written += block_size_bytes;
    }

    fclose(fp);

    // bytes_written 과 total_bytes 가 정확히 일치할 것으로 기대
    if (bytes_written != total_bytes)
    {
        fprintf(stderr, "Warning: bytes_written(%zu) != total_bytes(%zu)\n",
                bytes_written, total_bytes);
    }

    int ret = write_binary_file(output_filename, out, bytes_written);

    free(out);
    free(dev_buf);
    free(block_ids);
    dict_free(&dict);

    return ret;
}
