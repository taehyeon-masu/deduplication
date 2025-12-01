# Time-series Deduplication Project

---

## 1. Build

프로젝트 root에서:

```bash
make
```

성공하면 ./dedup_bin 실행 파일이 생성됩니다.

---

## 2. Benchmark 실행

프로젝트 root에서:

Encoding Benchmark

```bash
sudo ./scripts/encode_bench.sh
```

Decoding Benchmark

```bash
sudo ./scripts/encode_bench.sh
```

결과는 ./results/decode_YYYYMMDD_HHMMSS/ 아래에 저장됩.
