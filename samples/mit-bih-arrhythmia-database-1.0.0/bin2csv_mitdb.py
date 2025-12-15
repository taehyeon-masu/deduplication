#!/usr/bin/env python3
import argparse
import numpy as np
import csv

def main():
    parser = argparse.ArgumentParser(
        description="Convert MIT-BIH combined binary (int16, 2ch) to CSV."
    )
    parser.add_argument("input_bin", help="input .bin file (e.g., mitdb_all_ch2_s16le.bin)")
    parser.add_argument("output_csv", help="output .csv file")
    parser.add_argument(
        "--channels", "-c", type=int, default=2,
        help="number of channels (default: 2 for MIT-BIH)"
    )
    parser.add_argument(
        "--fs", type=float, default=360.0,
        help="sampling rate in Hz (default: 360 for MIT-BIH)"
    )
    args = parser.parse_args()

    # int16 little-endian: '<i2'
    try:
        data = np.fromfile(args.input_bin, dtype="<i2")
    except Exception as e:
        raise SystemExit(f"Failed to read binary file: {e}")

    if data.size % args.channels != 0:
        raise SystemExit(
            f"Data length ({data.size}) is not divisible by channels ({args.channels})."
        )

    num_samples = data.size // args.channels
    samples = data.reshape(num_samples, args.channels)

    with open(args.output_csv, "w", newline="") as f:
        writer = csv.writer(f)

        # header: idx, time_sec, ch0, ch1, ...
        header = ["idx", "time_sec"] + [f"ch{ch}" for ch in range(args.channels)]
        writer.writerow(header)

        if args.fs > 0:
            for i in range(num_samples):
                t = i / args.fs
                row = [i, t] + samples[i].tolist()
                writer.writerow(row)
        else:
            # fs <= 0 인 경우 time_sec 없이 저장
            writer.writerow(["(time disabled because fs<=0)"])
            for i in range(num_samples):
                row = [i, ""] + samples[i].tolist()
                writer.writerow(row)

    print(
        f"Done. Wrote {num_samples} samples, {args.channels} channels "
        f"to {args.output_csv}"
    )

if __name__ == "__main__":
    main()

