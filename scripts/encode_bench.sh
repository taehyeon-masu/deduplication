#!/bin/bash
# Run with: sudo ./scripts/encode_bench.sh
set -euo pipefail

DEDUP_BIN=./dedup_bin
SAMPLES_DIR=./samples
RESULT_ROOT=./results

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
ENC_ROOT="${RESULT_ROOT}/encode_${TIMESTAMP}"

mkdir -p "${ENC_ROOT}"

# 요약 CSV 파일 (모든 센서+블록 사이즈 결과 한 곳에)
SUMMARY_CSV="${ENC_ROOT}/encode_summary.csv"
echo "mode,sensor,block_size,elapsed_sec,max_rss_kb" > "${SUMMARY_CSV}"

# 센서별 파일 이름과 width_bytes 매핑
# 형식: "파일명:width_bytes"
DATASETS=(
  "T_raw.bin:2"
  "RH_raw.bin:2"
  "lux_raw.bin:2"
  "P_raw.bin:4"
)

# block_size_samples 리스트 (2,4,...,32)
BLOCK_SIZES=(2 4 6 8 10 12 14 16 18 20 22 24 26 28 30 32)

ulimit -n 999999

echo "Encode benchmark start: ${TIMESTAMP}"
echo "Results will be stored under: ${ENC_ROOT}"
echo

for entry in "${DATASETS[@]}"; do
  IFS=":" read -r FNAME WIDTH <<< "${entry}"
  SENSOR="${FNAME%_raw.bin}"     # T, RH, lux, P
  INPUT="${SAMPLES_DIR}/${FNAME}"

  for B in "${BLOCK_SIZES[@]}"; do
    BLOCK_DIR="${ENC_ROOT}/${SENSOR}/b$(printf "%02d" "${B}")"
    mkdir -p "${BLOCK_DIR}"

    OUTPUT_DDP="${BLOCK_DIR}/${SENSOR}_b${B}.ddp"
    DSTAT_CSV="${BLOCK_DIR}/dstat.csv"
    PERF_LOG="${BLOCK_DIR}/perf.txt"
    TIME_LOG="${BLOCK_DIR}/time.txt"

    echo "=== [ENC] sensor=${SENSOR}, width=${WIDTH}, block_size_samples=${B} ==="
    echo "Input : ${INPUT}"
    echo "Output: ${OUTPUT_DDP}"
    echo

    # 캐시 드랍 (dc1,2,3)
    echo 1 > /proc/sys/vm/drop_caches
    echo 2 > /proc/sys/vm/drop_caches
    echo 3 > /proc/sys/vm/drop_caches

    # dstat: CSV 출력 (1초 간격)
    dstat -cdngym --output "${DSTAT_CSV}" 1 >/dev/null 2>&1 &
    DSTAT_PID=$!

    # perf + time + dedup_bin
    #  - perf stat 결과는 PERF_LOG
    #  - /usr/bin/time의 상세 결과는 TIME_LOG
    /usr/bin/time -v -o "${TIME_LOG}" \
      perf stat -d -o "${PERF_LOG}" \
        "${DEDUP_BIN}" c "${WIDTH}" "${B}" "${INPUT}" "${OUTPUT_DDP}"

    # dstat 종료
    sleep 2
    kill "${DSTAT_PID}" >/dev/null 2>&1 || true
    wait "${DSTAT_PID}" 2>/dev/null || true

    # perf/time 로그에서 요약 값 추출
    # perf: "X.XXXXX seconds time elapsed" -> X.XXXXX
    elapsed=$(awk '/seconds time elapsed/ {print $1}' "${PERF_LOG}" | head -n1)
    # time: "Maximum resident set size (kbytes): 12345" -> 12345
    max_rss_kb=$(awk -F: '/Maximum resident set size/ {gsub(/^[ \t]+/, "", $2); print $2}' "${TIME_LOG}" | head -n1)

    # CSV에 한 줄 추가
    echo "ENC,${SENSOR},${B},${elapsed},${max_rss_kb}" >> "${SUMMARY_CSV}"

    echo "--- done [ENC] sensor=${SENSOR}, block_size_samples=${B} ---"
    echo "  elapsed_sec=${elapsed}, max_rss_kb=${max_rss_kb}"
    echo
  done
done

echo "All encode benchmarks finished."
echo "Encode results root: ${ENC_ROOT}"
echo "Summary CSV: ${SUMMARY_CSV}"
