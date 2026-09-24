#!/usr/bin/env python3
"""Compare two trajectory CSVs (t_s, x_enu_m, y_enu_m, z_enu_m, ...).

The candidate is sampled at the reference times by linear interpolation over
their common time span. Reports RMS and maximum position difference.

  compare_trajectories.py reference.csv candidate.csv [--max-rms 0.05]

With --exact the two files must contain identical rows at identical times
(used to check that repeated Gazebo runs are deterministic).
"""

import argparse
import csv
import math
import sys


def load(path):
    with open(path, newline='', encoding='utf-8') as stream:
        rows = list(csv.reader(stream))
    return [tuple(float(v) for v in row) for row in rows[1:]]


def interpolate(rows, t):
    for a, b in zip(rows, rows[1:]):
        if a[0] <= t <= b[0]:
            if b[0] == a[0]:
                return a
            f = (t - a[0]) / (b[0] - a[0])
            return tuple(av + f * (bv - av) for av, bv in zip(a, b))
    return None


def compare_exact(reference, candidate):
    by_time = {round(r[0], 6): r for r in candidate}
    common = [r for r in reference if round(r[0], 6) in by_time]
    mismatches = [r for r in common if by_time[round(r[0], 6)] != r]
    print(f'{len(common)} common timestamps, {len(mismatches)} differ')
    return bool(common) and not mismatches


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('reference')
    parser.add_argument('candidate')
    parser.add_argument('--max-rms', type=float, default=None)
    parser.add_argument('--exact', action='store_true')
    args = parser.parse_args()

    reference, candidate = load(args.reference), load(args.candidate)
    if args.exact:
        ok = compare_exact(reference, candidate)
        print('RESULT:', 'IDENTICAL' if ok else 'DIFFERENT')
        return 0 if ok else 1

    t_lo, t_hi = candidate[0][0], candidate[-1][0]
    errors = []
    for row in reference:
        if t_lo <= row[0] <= t_hi:
            other = interpolate(candidate, row[0])
            errors.append((row[0], math.dist(row[1:4], other[1:4])))
    rms = math.sqrt(sum(e * e for _, e in errors) / len(errors))
    t_max, e_max = max(errors, key=lambda e: e[1])
    print(f'compared {len(errors)} samples over t = {errors[0][0]:.2f} .. '
          f'{errors[-1][0]:.2f} s')
    print(f'position difference: RMS {rms:.4f} m, max {e_max:.4f} m '
          f'at t = {t_max:.2f} s')
    if args.max_rms is not None:
        ok = rms <= args.max_rms
        print('RESULT:', 'PASS' if ok else 'FAIL', f'(RMS limit {args.max_rms} m)')
        return 0 if ok else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
