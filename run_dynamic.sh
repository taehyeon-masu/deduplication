#!/bin/bash
set -e

IN="./samples/weather_7days.bin"
OUT_DIR="./results"
mkdir -p "${OUT_DIR}"

# 9채널, 각 2바이트
NUM_SEGS=9
SEG_BYTES=(2 2 2 2 2 2 2 2 2)

for N in $(seq 2 2 144); do
    mkdir ${OUT_DIR}/weather_7days_${N}
    OUT="${OUT_DIR}/weather_7days_${N}/weather_7days${N}.ddp"

    echo "=== Compress with DEV_TOP_N=${N} -> ${OUT} ==="

    ./dedup_bin ddp1 \
        "${NUM_SEGS}" \
        "${SEG_BYTES[@]}" \
        "${N}" \
        "${IN}" \
        "${OUT}"
done
