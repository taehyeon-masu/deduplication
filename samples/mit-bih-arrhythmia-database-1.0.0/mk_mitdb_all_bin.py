#!/usr/bin/env python3
import wfdb
import numpy as np
from pathlib import Path

# 현재 스크립트가 있는 디렉토리 (= MIT-BIH 파일들이 있는 곳)
MITDB_DIR = Path(__file__).parent

# 출력 바이너리 파일 이름
OUT_BIN = MITDB_DIR / "mitdb_all.bin"

def main():
    # .hea 파일들 기준으로 record 이름 수집
    records = sorted({p.stem for p in MITDB_DIR.glob("*.hea")})
    print("Found records:", records)

    total_values = 0
    total_bytes = 0

    with open(OUT_BIN, "wb") as fout:
        for rec_name in records:
            # ★ 로컬 파일은 pn_dir 쓰지 말고, full path를 record_name으로!
            rec_path = MITDB_DIR / rec_name

            rec = wfdb.rdrecord(
                str(rec_path),   # ex) "/home/.../mit-bih-.../100"
                physical=False   # 디지털 값(d_signal)으로 읽기
            )

            if rec.d_signal is None:
                raise RuntimeError(
                    f"Record {rec_name}: d_signal is None "
                    "(check physical=False or files)."
                )

            # d_signal: (num_samples, num_channels)
            d = rec.d_signal.astype(np.int16)  # 2 bytes per sample per channel

            # row-major 그대로 파일에 write → 샘플별 interleave
            d.tofile(fout)

            nsamp, nch = d.shape
            vals = d.size
            bytes_written = vals * 2

            total_values += vals
            total_bytes += bytes_written

            print(
                f"{rec_name}: {nsamp} samples, {nch} channels → "
                f"{vals} int16 values ({bytes_written} bytes)"
            )

    print()
    print(f"Written {OUT_BIN}")
    print(f"  total values : {total_values}")
    print(f"  total bytes  : {total_bytes} (≈ {total_bytes/1024/1024:.2f} MiB)")

if __name__ == "__main__":
    main()

