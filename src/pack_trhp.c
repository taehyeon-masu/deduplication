#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*
 * Usage:
 *   ./pack_trhp T_raw.bin RH_raw.bin lux_raw.bin P_raw.bin combined.bin
 *
 * 각 입력 파일 포맷:
 *   - T_raw.bin  : 2-byte little-endian 샘플들
 *   - RH_raw.bin : 2-byte little-endian 샘플들
 *   - lux_raw.bin: 2-byte little-endian 샘플들 (1초 주기)
 *   - P_raw.bin  : 4-byte little-endian 샘플들 (1초 주기)
 *
 * 2초 주기 하나의 block 구성:
 *   [ T(2B), RH(2B), lux_1(2B), P_1(4B), lux_2(2B), P_2(4B) ] = 총 16 bytes
 *
 * 어느 하나라도 부족해서 위 16바이트를 못 채우는 순간,
 * 그 이후 남은 샘플들은 버리고 종료.
 */

int main(int argc, char *argv[])
{
    if (argc != 6) {
        fprintf(stderr,
                "Usage: %s T_raw.bin RH_raw.bin lux_raw.bin P_raw.bin combined.bin\n",
                argv[0]);
        return 1;
    }

    const char *t_file   = argv[1];
    const char *rh_file  = argv[2];
    const char *lux_file = argv[3];
    const char *p_file   = argv[4];
    const char *out_file = argv[5];

    FILE *ft   = fopen(t_file,   "rb");
    FILE *frh  = fopen(rh_file,  "rb");
    FILE *flux = fopen(lux_file, "rb");
    FILE *fp   = fopen(p_file,   "rb");
    FILE *fout = fopen(out_file, "wb");

    if (!ft || !frh || !flux || !fp || !fout) {
        fprintf(stderr, "Failed to open one or more files:\n");
        if (!ft)   fprintf(stderr, "  T   : %s\n", t_file);
        if (!frh)  fprintf(stderr, "  RH  : %s\n", rh_file);
        if (!flux) fprintf(stderr, "  lux : %s\n", lux_file);
        if (!fp)   fprintf(stderr, "  P   : %s\n", p_file);
        if (!fout) fprintf(stderr, "  out : %s\n", out_file);
        goto cleanup;
    }

    unsigned char bufT[2];
    unsigned char bufRH[2];
    unsigned char bufLux1[2];
    unsigned char bufLux2[2];
    unsigned char bufP1[4];
    unsigned char bufP2[4];

    size_t blocks_written = 0;

    for (;;) {
        // 2초 주기 하나를 구성할 샘플들 읽기
        size_t nT   = fread(bufT,   1, 2, ft);
        size_t nRH  = fread(bufRH,  1, 2, frh);
        size_t nL1  = fread(bufLux1,1, 2, flux);
        size_t nP1  = fread(bufP1,  1, 4, fp);
        size_t nL2  = fread(bufLux2,1, 2, flux);
        size_t nP2  = fread(bufP2,  1, 4, fp);

        // 하나라도 못 읽으면(EOF 포함) 더 이상 block을 만들 수 없음 → 종료
        if (nT  < 2 || nRH < 2 ||
            nL1 < 2 || nP1 < 4 ||
            nL2 < 2 || nP2 < 4) {
            break;
        }

        // 순서: T, RH, lux_1, P_1, lux_2, P_2
        if (fwrite(bufT,   1, 2, fout) != 2) goto write_error;
        if (fwrite(bufRH,  1, 2, fout) != 2) goto write_error;
        if (fwrite(bufLux1,1, 2, fout) != 2) goto write_error;
        if (fwrite(bufP1,  1, 4, fout) != 4) goto write_error;
        if (fwrite(bufLux2,1, 2, fout) != 2) goto write_error;
        if (fwrite(bufP2,  1, 4, fout) != 4) goto write_error;

        blocks_written++;
    }

    printf("Packed %zu blocks (each 16 bytes) into %s\n",
           blocks_written, out_file);

    goto cleanup;

write_error:
    fprintf(stderr, "Write error while writing to %s\n", out_file);

cleanup:
    if (ft)   fclose(ft);
    if (frh)  fclose(frh);
    if (flux) fclose(flux);
    if (fp)   fclose(fp);
    if (fout) fclose(fout);

    return 0;
}
