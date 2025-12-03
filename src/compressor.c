#include "../include/compressor.h"
#include "../include/dictionary.h"
#include "../include/bin_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

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
 * Multi-layout (seg_sizes[] 로 블록 레이아웃 지정)
 *   magic: "DDP1"
 * ============================================================ */

static size_t compute_block_bytes_multi(int num_segs, const int *seg_sizes)
{
    size_t sum = 0;
    for (int i = 0; i < num_segs; ++i)
    {
        if (seg_sizes[i] <= 0)
            return 0;
        sum += (size_t)seg_sizes[i];
    }
    return sum;
}

static size_t compute_dev_len_multi(int num_segs, const int *seg_sizes)
{
    size_t dev = 0;
    for (int s = 0; s < num_segs; ++s)
    {
        int len = seg_sizes[s];
        int take = len / 2;
        int extra = len % 2;
        if (extra && s == num_segs - 1)
            take += 1;
        dev += (size_t)take;
    }
    return dev;
}

/* 각 세그먼트(센서 sample 단위)에서 하위 바이트를 deviation으로 */
static size_t extract_base_and_deviation_multi(
    unsigned char *block_buf,
    int num_segs,
    const int *seg_sizes,
    unsigned char *dev_buf)
{
    size_t dev_off = 0;
    size_t offset = 0;

    for (int s = 0; s < num_segs; ++s)
    {
        int len = seg_sizes[s];
        unsigned char *seg = block_buf + offset;
        offset += (size_t)len;

        int take = len / 2;
        int extra = len % 2;
        if (extra && s == num_segs - 1)
            take += 1;

        if (take > 0)
        {
            memcpy(dev_buf + dev_off, seg, (size_t)take);
            memset(seg, 0, (size_t)take);
            dev_off += (size_t)take;
        }
    }
    return dev_off;
}

/* base_block + dev_buf → out_block (multi-layout) */
static size_t merge_base_and_deviation_multi(
    const unsigned char *base_block,
    int num_segs,
    const int *seg_sizes,
    const unsigned char *dev_buf,
    unsigned char *out_block)
{
    size_t dev_off = 0;
    size_t offset = 0;

    for (int s = 0; s < num_segs; ++s)
    {
        int len = seg_sizes[s];
        unsigned char *out_seg = out_block + offset;
        const unsigned char *base_seg = base_block + offset;
        offset += (size_t)len;

        int take = len / 2;
        int extra = len % 2;
        if (extra && s == num_segs - 1)
            take += 1;

        if (take > 0)
        {
            memcpy(out_seg, dev_buf + dev_off, (size_t)take);
            dev_off += (size_t)take;
        }

        int remain = len - take;
        if (remain > 0)
        {
            memcpy(out_seg + take, base_seg + take, (size_t)remain);
        }
    }
    return dev_off;
}

/* ------------------------------------------------------------
 * multi-layout 압축 (DDP1)
 *
 * 포맷:
 *  magic: 'D','D','P','1'
 *  u32: block_bytes           (한 블록의 총 바이트 수)
 *  u32: num_segs              (블록 내부 세그먼트 개수)
 *  u32: dict_size
 *  u32: num_blocks
 *  u32[num_segs]: seg_sizes
 *  [dictionary]: dict_size * block_bytes bytes
 *  [block_ids]:  num_blocks * 4 bytes
 *  [deviation]:  num_blocks * dev_len_per_block bytes
 * ------------------------------------------------------------ */
int compress_file(const char *input_filename,
                  const char *output_filename,
                  int num_segs,
                  const int *seg_sizes)
{
    if (num_segs <= 0 || !seg_sizes)
    {
        fprintf(stderr, "compress_file_multi: invalid num_segs\n");
        return 1;
    }

    size_t block_bytes = compute_block_bytes_multi(num_segs, seg_sizes);
    if (block_bytes == 0)
    {
        fprintf(stderr, "compress_file_multi: invalid seg_sizes (sum == 0 or negative)\n");
        return 1;
    }

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

    size_t num_blocks = nbytes / block_bytes;
    if (num_blocks == 0)
    {
        fprintf(stderr, "No full blocks found (multi-layout)\n");
        fclose(fin);
        return 1;
    }

    size_t used_bytes = num_blocks * block_bytes;
    if (used_bytes < nbytes)
    {
        fprintf(stderr,
                "Warning: last %zu bytes are ignored (not enough to fill a block)\n",
                (nbytes - used_bytes));
    }

    uint32_t *block_ids = (uint32_t *)malloc(sizeof(uint32_t) * num_blocks);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids\n");
        fclose(fin);
        return 1;
    }

    size_t dev_len_per_block = compute_dev_len_multi(num_segs, seg_sizes);
    unsigned char *dev_stream = (unsigned char *)malloc(num_blocks * dev_len_per_block);
    if (!dev_stream)
    {
        fprintf(stderr, "Failed to allocate dev_stream (multi)\n");
        free(block_ids);
        fclose(fin);
        return 1;
    }

    Dictionary dict;
    dict_init(&dict, block_bytes);

    unsigned char *block_buf = (unsigned char *)malloc(block_bytes);
    unsigned char *dev_buf = (unsigned char *)malloc(dev_len_per_block);
    if (!block_buf || !dev_buf)
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

    for (size_t b = 0; b < num_blocks; ++b)
    {
        size_t n = fread(block_buf, 1, block_bytes, fin);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read block %zu (multi)\n", b);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        size_t used_dev = extract_base_and_deviation_multi(
            block_buf, num_segs, seg_sizes, dev_buf);
        if (used_dev != dev_len_per_block)
        {
            fprintf(stderr, "extract_base_and_deviation_multi: used_dev=%zu, expected=%zu\n",
                    used_dev, dev_len_per_block);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            fclose(fin);
            return 1;
        }

        memcpy(dev_stream + b * dev_len_per_block, dev_buf, dev_len_per_block);

        int idx = dict_find(&dict, block_buf);
        if (idx == -1)
        {
            idx = dict_add(&dict, block_buf);
        }
        block_ids[b] = (uint32_t)idx;
    }

    fclose(fin);

    FILE *fp = fopen(output_filename, "wb");
    if (!fp)
    {
        perror("fopen output");
        free(block_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        return 1;
    }

    const unsigned char magic[4] = {'D', 'D', 'P', '1'};
    if (fwrite(magic, 1, 4, fp) != 4)
    {
        fprintf(stderr, "Failed to write magic (DDP1)\n");
        fclose(fp);
        free(block_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        return 1;
    }

    uint32_t block_bytes_u32 = (uint32_t)block_bytes;
    uint32_t num_segs_u32 = (uint32_t)num_segs;
    uint32_t dict_size_u32 = (uint32_t)dict.size;
    uint32_t num_blocks_u32 = (uint32_t)num_blocks;

    if (!write_u32_le(fp, block_bytes_u32) ||
        !write_u32_le(fp, num_segs_u32) ||
        !write_u32_le(fp, dict_size_u32) ||
        !write_u32_le(fp, num_blocks_u32))
    {
        fprintf(stderr, "Failed to write DDP1 header (block_bytes/num_segs/dict_size/num_blocks)\n");
        fclose(fp);
        free(block_buf);
        free(dev_buf);
        free(dev_stream);
        free(block_ids);
        dict_free(&dict);
        return 1;
    }

    for (int s = 0; s < num_segs; ++s)
    {
        uint32_t v = (uint32_t)seg_sizes[s];
        if (!write_u32_le(fp, v))
        {
            fprintf(stderr, "Failed to write seg_sizes[%d]\n", s);
            fclose(fp);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            return 1;
        }
    }

    for (int i = 0; i < dict.size; ++i)
    {
        if (fwrite(dict.blocks[i], 1, block_bytes, fp) != block_bytes)
        {
            fprintf(stderr, "Failed to write dictionary block %d (multi)\n", i);
            fclose(fp);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            return 1;
        }
    }

    for (size_t b = 0; b < num_blocks; ++b)
    {
        if (!write_u32_le(fp, block_ids[b]))
        {
            fprintf(stderr, "Failed to write block id %zu (multi)\n", b);
            fclose(fp);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            return 1;
        }
    }

    size_t dev_total_bytes = num_blocks * dev_len_per_block;
    if (dev_total_bytes > 0)
    {
        if (fwrite(dev_stream, 1, dev_total_bytes, fp) != dev_total_bytes)
        {
            fprintf(stderr, "Failed to write deviation stream (multi)\n");
            fclose(fp);
            free(block_buf);
            free(dev_buf);
            free(dev_stream);
            free(block_ids);
            dict_free(&dict);
            return 1;
        }
    }

    fclose(fp);
    free(block_buf);
    free(dev_buf);
    free(dev_stream);
    free(block_ids);
    dict_free(&dict);

    fprintf(stderr,
            "Compressed (DDP1 multi): used_bytes=%zu, block_bytes=%zu, dict_size=%d, num_blocks=%zu\n",
            used_bytes, block_bytes, dict.size, num_blocks);

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
        fprintf(stderr, "Failed to read magic (multi dec)\n");
        fclose(fp);
        return 1;
    }
    if (!(magic[0] == 'D' && magic[1] == 'D' &&
          magic[2] == 'P' && magic[3] == '1'))
    {
        fprintf(stderr, "Invalid magic for DDP1 in decompress_file_multi\n");
        fclose(fp);
        return 1;
    }

    uint32_t block_bytes_u32;
    uint32_t num_segs_u32;
    uint32_t dict_size_u32;
    uint32_t num_blocks_u32;

    if (!read_u32_le(fp, &block_bytes_u32) ||
        !read_u32_le(fp, &num_segs_u32) ||
        !read_u32_le(fp, &dict_size_u32) ||
        !read_u32_le(fp, &num_blocks_u32))
    {
        fprintf(stderr, "Failed to read DDP1 header (block_bytes/num_segs/dict_size/num_blocks)\n");
        fclose(fp);
        return 1;
    }

    size_t block_bytes = (size_t)block_bytes_u32;
    int num_segs = (int)num_segs_u32;
    size_t dict_size = (size_t)dict_size_u32;
    size_t num_blocks = (size_t)num_blocks_u32;

    if (num_segs <= 0)
    {
        fprintf(stderr, "Invalid num_segs in DDP1 header\n");
        fclose(fp);
        return 1;
    }

    int *seg_sizes = (int *)malloc(sizeof(int) * num_segs);
    if (!seg_sizes)
    {
        fprintf(stderr, "Failed to allocate seg_sizes (dec)\n");
        fclose(fp);
        return 1;
    }

    size_t sum_bytes = 0;
    for (int s = 0; s < num_segs; ++s)
    {
        uint32_t v;
        if (!read_u32_le(fp, &v))
        {
            fprintf(stderr, "Failed to read seg_sizes[%d] in DDP1\n", s);
            free(seg_sizes);
            fclose(fp);
            return 1;
        }
        seg_sizes[s] = (int)v;
        sum_bytes += (size_t)v;
    }

    if (sum_bytes != block_bytes)
    {
        fprintf(stderr,
                "Warning: sum(seg_sizes)=%zu != block_bytes=%zu in header\n",
                sum_bytes, block_bytes);
    }

    size_t dev_len_per_block = compute_dev_len_multi(num_segs, seg_sizes);

    Dictionary dict;
    dict_init(&dict, block_bytes);

    for (size_t i = 0; i < dict_size; ++i)
    {
        unsigned char *buf = (unsigned char *)malloc(block_bytes);
        if (!buf)
        {
            fprintf(stderr, "Failed to allocate block buffer (dict read, multi)\n");
            free(seg_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        size_t n = fread(buf, 1, block_bytes, fp);
        if (n != block_bytes)
        {
            fprintf(stderr, "Failed to read dictionary block %zu (multi)\n", i);
            free(buf);
            free(seg_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
        dict_add(&dict, buf);
        free(buf);
    }

    uint32_t *block_ids = (uint32_t *)malloc(sizeof(uint32_t) * num_blocks);
    if (!block_ids)
    {
        fprintf(stderr, "Failed to allocate block_ids (multi dec)\n");
        free(seg_sizes);
        dict_free(&dict);
        fclose(fp);
        return 1;
    }
    for (size_t b = 0; b < num_blocks; ++b)
    {
        if (!read_u32_le(fp, &block_ids[b]))
        {
            fprintf(stderr, "Failed to read block id %zu (multi dec)\n", b);
            free(block_ids);
            free(seg_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
    }

    size_t dev_total_bytes = num_blocks * dev_len_per_block;
    unsigned char *dev_stream = NULL;
    if (dev_total_bytes > 0)
    {
        dev_stream = (unsigned char *)malloc(dev_total_bytes);
        if (!dev_stream)
        {
            fprintf(stderr, "Failed to allocate dev_stream (multi dec)\n");
            free(block_ids);
            free(seg_sizes);
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
            free(seg_sizes);
            dict_free(&dict);
            fclose(fp);
            return 1;
        }
    }

    fclose(fp);

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
        free(seg_sizes);
        dict_free(&dict);
        return 1;
    }

    for (size_t b = 0; b < num_blocks; ++b)
    {
        uint32_t id = block_ids[b];
        if (id >= (uint32_t)dict.size)
        {
            fprintf(stderr, "Invalid dictionary id %u at block %zu (multi dec)\n", id, b);
            free(out);
            free(tmp);
            free(dev_stream);
            free(block_ids);
            free(seg_sizes);
            dict_free(&dict);
            return 1;
        }
        const unsigned char *base_block = dict.blocks[id];
        const unsigned char *dev_ptr = dev_stream ? (dev_stream + b * dev_len_per_block) : NULL;

        size_t used_dev = 0;
        if (dev_ptr)
        {
            used_dev = merge_base_and_deviation_multi(
                base_block, num_segs, seg_sizes, dev_ptr, tmp);
        }
        else
        {
            memcpy(tmp, base_block, block_bytes);
        }

        if (dev_ptr && used_dev != dev_len_per_block)
        {
            fprintf(stderr, "merge used_dev=%zu != dev_len_per_block=%zu (multi dec)\n",
                    used_dev, dev_len_per_block);
            free(out);
            free(tmp);
            free(dev_stream);
            free(block_ids);
            free(seg_sizes);
            dict_free(&dict);
            return 1;
        }

        size_t offset = b * block_bytes;
        if (offset + block_bytes > total_bytes)
        {
            fprintf(stderr, "Output buffer overflow risk (multi dec)\n");
            free(out);
            free(tmp);
            free(dev_stream);
            free(block_ids);
            free(seg_sizes);
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
    free(seg_sizes);
    dict_free(&dict);

    return ret;
}
