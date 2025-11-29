#!/bin/bash
# Run with: sudo ./scripts/decode_bench.sh
set -euo pipefail

DEDUP_BIN=./dedup_bin
RESULT_ROOT=./results

# 가장 최근 encode_* 디렉토리 찾기
LATEST_ENC_ROOT=$(ls -d "${RESULT_ROOT}"/encode_* 2>/dev/null | sort | tail -n1 || true)

if [[ -z "${LATEST_ENC_ROOT}" ]]; then
  echo "No encode_* directory found under ${RESULT_ROOT}"
  exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DEC_ROOT="${RESULT_ROOT}/decode_${TIMESTAMP}"

mkdir -p "${DEC_ROOT}"

# 요약 CSV 파일
SUMMARY_CSV="${DEC_ROOT}/decode_summary.csv"
echo "mode,sensor,block_size,elapsed_sec,max_rss_kb" > "${SUMMARY_CSV}"

# 센서 이름 & block size 리스트 (encode 스크립트와 동일)
SENSORS=(T RH lux P)
BLOCK_SIZES=(2 4 6 8 10 12 14 16 18 20 22 24 26 28 30 32)

ulimit -n 999999

echo "Decode benchmark start: ${TIMESTAMP}"
echo "Using encoded data from: ${LATEST_ENC_ROOT}"
echo "Results will be stored under: ${DEC_ROOT}"
echo

for SENSOR in "${SENSORS[@]}"; do
  for B in "${BLOCK_SIZES[@]}"; do
    ENC_BLOCK_DIR="${LATEST_ENC_ROOT}/${SENSOR}/b$(printf "%02d" "${B}")"
    INPUT_DDP="${ENC_BLOCK_DIR}/${SENSOR}_b${B}.ddp"

    if [[ ! -f "${INPUT_DDP}" ]]; then
      echo "Skip [DEC] sensor=${SENSOR}, B=${B} (no ddp: ${INPUT_DDP})"
      continue
    fi

    DEC_BLOCK_DIR="${DEC_ROOT}/${SENSOR}/b$(printf "%02d" "${B}")"
    mkdir -p "${DEC_BLOCK_DIR}"

    OUTPUT_BIN="${DEC_BLOCK_DIR}/${SENSOR}_b${B}.bin"
    DSTAT_CSV="${DEC_BLOCK_DIR}/dstat.csv"
    PERF_LOG="${DEC_BLOCK_DIR}/perf.txt"
    TIME_LOG="${DEC_BLOCK_DIR}/time.txt"

    echo "=== [DEC] sensor=${SENSOR}, block_size_samples=${B} ==="
    echo "Input : ${INPUT_DDP}"
    echo "Output: ${OUTPUT_BIN}"
    echo

    # 캐시 드랍
    echo 1 > /proc/sys/vm/drop_caches
    echo 2 > /proc/sys/vm/drop_caches
    echo 3 > /proc/sys/vm/drop_caches

    # dstat
    dstat -cdngym --output "${DSTAT_CSV}" 1 >/dev/null 2>&1 &
    DSTAT_PID=$!

    # perf + time + dedup_bin (decode)
    /usr/bin/time -v -o "${TIME_LOG}" \
      perf stat -d -o "${PERF_LOG}" \
        "${DEDUP_BIN}" d "${INPUT_DDP}" "${OUTPUT_BIN}"

    # dstat 종료
    sleep 2
    kill "${DSTAT_PID}" >/dev/null 2>&1 || true
    wait "${DSTAT_PID}" 2>/dev/null || true

    # perf/time 로그에서 요약 값 추출
    elapsed=$(awk '/seconds time elapsed/ {print $1}' "${PERF_LOG}" | head -n1)
    max_rss_kb=$(awk -F: '/Maximum resident set size/ {gsub(/^[ \t]+/, "", $2); print $2}' "${TIME_LOG}" | head -n1)

    # CSV에 한 줄 추가
    echo "DEC,${SENSOR},${B},${elapsed},${max_rss_kb}" >> "${SUMMARY_CSV}"

    echo "--- done [DEC] sensor=${SENSOR}, block_size_samples=${B} ---"
    echo "  elapsed_sec=${elapsed}, max_rss_kb=${max_rss_kb}"
    echo
  done
done

echo "All decode benchmarks finished."
echo "Decode results root: ${DEC_ROOT}"
echo "Summary CSV: ${SUMMARY_CSV}"
