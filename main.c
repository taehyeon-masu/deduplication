#include "../include/compressor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage:\n"
            "  DDP1 compress (multi-layout, multi-sensor):\n"
            "    %s ddp1 <num_segs> <seg1_bytes> ... <segN_bytes> <DEV_TOP_N> <input.bin> <output.ddp>\n"
            "      - num_segs: 한 블록 안에 들어가는 세그먼트 개수\n"
            "      - seg*_bytes: 각 세그먼트의 바이트 수\n"
            "        예) T(2B), RH(2B), lux1(2B), P1(4B), lux2(2B), P2(4B)\n"
            "            => num_segs=6, segs=2 2 2 4 2 4\n"
            "      - DEV_TOP_N: deviation으로 뺄 비트 개수 (0이면 dev 없음)\n"
            "\n"
            "  Decompress (DDP1):\n"
            "    %s d <input.ddp> <output.bin>\n",
            prog, prog);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        usage(argv[0]);
        return 1;
    }

    /* ----- DDP1 Compress (multi-layout / multi-sensor) ----- */
    if (strcmp(argv[1], "ddp1") == 0)
    {
        if (argc < 7)
        {
            fprintf(stderr, "Invalid arguments for DDP1 (multi-layout) compress\n");
            usage(argv[0]);
            return 1;
        }

        int num_segs = atoi(argv[2]);
        if (num_segs <= 0)
        {
            fprintf(stderr, "num_segs must be positive\n");
            return 1;
        }

        /*
         * 인자 구조:
         *   argv[0] : prog
         *   argv[1] : "ddp1"
         *   argv[2] : num_segs
         *   argv[3..(3+num_segs-1)] : seg_sizes[]
         *   argv[3+num_segs]         : DEV_TOP_N
         *   argv[4+num_segs]         : input.bin
         *   argv[5+num_segs]         : output.ddp
         *
         * 전체 개수 = 6 + num_segs
         */
        int expected_argc = 6 + num_segs;
        if (argc != expected_argc)
        {
            fprintf(stderr,
                    "Expected %d arguments for DDP1 compress, got %d\n",
                    expected_argc, argc);
            usage(argv[0]);
            return 1;
        }

        int *seg_sizes = (int *)malloc(sizeof(int) * num_segs);
        if (!seg_sizes)
        {
            fprintf(stderr, "Failed to allocate seg_sizes\n");
            return 1;
        }

        for (int i = 0; i < num_segs; ++i)
        {
            seg_sizes[i] = atoi(argv[3 + i]);
            if (seg_sizes[i] <= 0)
            {
                fprintf(stderr, "Invalid seg_sizes[%d] = %d\n", i, seg_sizes[i]);
                free(seg_sizes);
                return 1;
            }
        }

        /* DEV_TOP_N */
        int dev_top_n = atoi(argv[3 + num_segs]);
        if (dev_top_n < 0)
        {
            fprintf(stderr, "DEV_TOP_N must be non-negative (0이면 deviation 미사용)\n");
            free(seg_sizes);
            return 1;
        }

        const char *input_file = argv[4 + num_segs];
        const char *output_ddp = argv[5 + num_segs];

        int ret = compress_file(input_file, output_ddp,
                                num_segs, seg_sizes, dev_top_n);
        free(seg_sizes);

        if (ret != 0)
        {
            fprintf(stderr, "compress_file_multi (DDP1) failed\n");
            return ret;
        }
        return 0;
    }
    /* ----- Decompress (DDP1) ----- */
    else if (strcmp(argv[1], "d") == 0 || strcmp(argv[1], "D") == 0)
    {
        if (argc != 4)
        {
            fprintf(stderr, "Invalid arguments for decompress\n");
            usage(argv[0]);
            return 1;
        }

        const char *input_ddp  = argv[2];
        const char *output_bin = argv[3];

        int ret = decompress_file(input_ddp, output_bin);
        if (ret != 0)
        {
            fprintf(stderr, "decompress_file failed\n");
            return ret;
        }
        return 0;
    }
    else
    {
        fprintf(stderr, "Unknown mode '%s'\n", argv[1]);
        usage(argv[0]);
        return 1;
    }
}
