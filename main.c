#include "./include/compressor.h"
#include <stdio.h>
#include <stdlib.h>

// 사용법:
//   압축:   ./dedup_bin c <width_bytes:1|2|4|8> <block_size_samples> <input.bin> <output.ddp>
//   복원:   ./dedup_bin d <input.ddp> <output.bin>
int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr,
                "Usage:\n"
                "  Compress:   %s c <width_bytes> <block_size_samples> <input.bin> <output.ddp>\n"
                "  Decompress: %s d <input.ddp> <output.bin>\n",
                argv[0], argv[0]);
        return 1;
    }

    char mode = argv[1][0];

    if (mode == 'c') {
        if (argc != 6) {
            fprintf(stderr,
                    "Usage: %s c <width_bytes> <block_size_samples> <input.bin> <output.ddp>\n",
                    argv[0]);
            return 1;
        }
        int width_bytes = atoi(argv[2]);
        int block_size_samples = atoi(argv[3]);
        const char *input_bin = argv[4];
        const char *output_ddp = argv[5];

        int ret = compress_file(input_bin, output_ddp,
                                width_bytes, block_size_samples);
        if (ret == 0) {
            printf("Compression succeeded.\n");
        } else {
            printf("Compression failed.\n");
        }
        return ret;

    } else if (mode == 'd') {
        if (argc != 4) {
            fprintf(stderr,
                    "Usage: %s d <input.ddp> <output.bin>\n",
                    argv[0]);
            return 1;
        }
        const char *input_ddp = argv[2];
        const char *output_bin = argv[3];

        int ret = decompress_file(input_ddp, output_bin);
        if (ret == 0) {
            printf("Decompression succeeded.\n");
        } else {
            printf("Decompression failed.\n");
        }
        return ret;

    } else {
        fprintf(stderr,
                "Unknown mode '%c'. Use 'c' (compress) or 'd' (decompress).\n",
                mode);
        return 1;
    }
}
