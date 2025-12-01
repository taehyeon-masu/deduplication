#include "../include/compressor.h"
#include "../include/dictionary.h"
#include "../include/bin_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

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

// 포맷:
//  magic: 'D','D','P','1' (4바이트)
//  u32: sample_count (압축에 사용된 샘플 수)
//  u32: block_size_samples
//  u8 : width_bytes
//  u8[3]: reserved(0)
//  u32: dict_size
//  u32: num_blocks
//  [dictionary]: dict_size * (block_size_samples * width_bytes) bytes
//  [block_ids]:  num_blocks * 4 bytes (u32, LE)

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

    unsigned char *data = NULL;
    size_t nbytes = 0;
    if (read_binary_file(input_filename, &data, &nbytes) != 0)
    {
        return 1;
    }

    if (nbytes < (size_t)width_bytes)
    {
        fprintf(stderr, "Input file too small\n");
        free(data);
        return 1;
    }

    size_t total_samples = nbytes / (size_t)width_bytes;
    if (total_samples == 0)
    {
        fprintf(stderr, "No full samples found\n");
        free(data);
        return 1;
    }

    size_t num_blocks = total_samples / (size_t)block_size_samples;
    if (num_blocks == 0)
    {
        fprintf(stderr,
                "Not enough samples for at least one full block "
                "(need >= %d samples)\n",
                block_size_samples);
        free(data);
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

    uint32_t *block_ids = (uint32_t *)malloc(sizeof(uint32_t) * num_blocks);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        free(data);
        return 1;
    }

    Dictionary dict;
    dict_init(&dict, block_size_bytes);

    // dedup: 블록 단위로 dictionary에 등록
    for (size_t b = 0; b < num_blocks; ++b)
    {
        size_t offset = b * block_size_bytes;
        const unsigned char *block_ptr = data + offset;

        int idx = dict_find(&dict, block_ptr);
        if (idx == -1)
        {
            idx = dict_add(&dict, block_ptr);
        }
        block_ids[b] = (uint32_t)idx;
    }

    FILE *fp = fopen(output_filename, "wb");
    if (!fp)
    {
        perror("fopen output");
        free(block_ids);
        dict_free(&dict);
        free(data);
        return 1;
    }

    const unsigned char magic[4] = {'D', 'D', 'P', '1'};
    if (fwrite(magic, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to write magic\n");
        fclose(fp);
        free(block_ids);
        dict_free(&dict);
        free(data);
        return 1;
    }

    uint32_t sample_count_u32 = (uint32_t)used_samples;
    uint32_t block_size_samples_u32 = (uint32_t)block_size_samples;
    uint32_t dict_size_u32 = (uint32_t)dict_size(&dict);
    uint32_t num_blocks_u32 = (uint32_t)num_blocks;

    if (!write_u32_le(fp, sample_count_u32) ||
        !write_u32_le(fp, block_size_samples_u32))
    {
        fprintf(stderr, "Failed to write header\n");
        fclose(fp);
        free(block_ids);
        dict_free(&dict);
        free(data);
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
        free(block_ids);
        dict_free(&dict);
        free(data);
        return 1;
    }

    if (!write_u32_le(fp, dict_size_u32) ||
        !write_u32_le(fp, num_blocks_u32))
    {
        fprintf(stderr, "Failed to write header tail\n");
        fclose(fp);
        free(block_ids);
        dict_free(&dict);
        free(data);
        return 1;
    }

    // dictionary 블록들: index 0..dict_size-1 순서대로 기록
    int dsz = dict_size(&dict);
    for (int i = 0; i < dsz; ++i)
    {
        const unsigned char *block = dict_get_block(&dict, i);
        if (!block)
        {
            fprintf(stderr, "dict_get_block(%d) returned NULL\n", i);
            fclose(fp);
            free(block_ids);
            dict_free(&dict);
            free(data);
            return 1;
        }
        if (fwrite(block, 1, block_size_bytes, fp) != block_size_bytes)
        {
            fprintf(stderr, "Failed to write dictionary block %d\n", i);
            fclose(fp);
            free(block_ids);
            dict_free(&dict);
            free(data);
            return 1;
        }
    }

    // block id stream
    for (size_t b = 0; b < num_blocks; ++b)
    {
        if (!write_u32_le(fp, block_ids[b]))
        {
            fprintf(stderr, "Failed to write block id %zu\n", b);
            fclose(fp);
            free(block_ids);
            dict_free(&dict);
            free(data);
            return 1;
        }
    }

    fclose(fp);
    free(block_ids);
    dict_free(&dict);
    free(data);

    fprintf(stderr,
            "Compressed: used_samples=%zu, block_size_samples=%d, dict_size=%u, num_blocks=%zu\n",
            used_samples, block_size_samples, dict_size_u32, num_blocks);

    return 0;
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
    size_t dict_size_header = (size_t)dict_size_u32;
    size_t num_blocks = (size_t)num_blocks_u32;
    size_t block_size_bytes = block_size_samples * (size_t)width_bytes;

    Dictionary dict;
    dict_init(&dict, block_size_bytes);

    // dictionary 블록 읽어서 사전에 추가
    for (size_t i = 0; i < dict_size_header; ++i)
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
        dict_add(&dict, buf); // 내부에서 복사
        free(buf);
    }

    // block id stream 읽기
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

    fclose(fp);

    size_t total_bytes = sample_count * (size_t)width_bytes;
    unsigned char *out = (unsigned char *)malloc(total_bytes);
    if (!out)
    {
        fprintf(stderr, "Failed to allocate output buffer\n");
        free(block_ids);
        dict_free(&dict);
        return 1;
    }

    size_t bytes_written = 0;
    for (size_t b = 0; b < num_blocks; ++b)
    {
        uint32_t id = block_ids[b];

        int dsz = dict_size(&dict);
        if ((int)id >= dsz)
        {
            fprintf(stderr, "Invalid dictionary id %u at block %zu\n", id, b);
            free(out);
            free(block_ids);
            dict_free(&dict);
            return 1;
        }

        const unsigned char *block = dict_get_block(&dict, (int)id);
        if (!block)
        {
            fprintf(stderr, "dict_get_block(%u) returned NULL at block %zu\n", id, b);
            free(out);
            free(block_ids);
            dict_free(&dict);
            return 1;
        }

        size_t to_copy = block_size_bytes;
        if (bytes_written + to_copy > total_bytes)
        {
            to_copy = total_bytes - bytes_written;
        }
        memcpy(out + bytes_written, block, to_copy);
        bytes_written += to_copy;
        if (bytes_written >= total_bytes)
            break;
    }

    int ret = write_binary_file(output_filename, out, bytes_written);

    free(out);
    free(block_ids);
    dict_free(&dict);

    return ret;
}
