#!/usr/bin/env python3
"""Generates a random balloon arena (same schema as config/arena_default.yaml).

Takes the rules (time limit, points, colours, vehicle contact outline,
balloon size) from a base arena file and draws new balloon positions,
reproducibly from a seed, under these constraints:

  spacing    the gap between any two balloon surfaces is at least the drone
             width (0.16 m). The contact outline is 0.155 m wide flying
             straight (0.19 m diagonally), so the drone fits between two
             balloons only with little margin and when aligned with the gap:
             horizontal centre distance >= balloon diameter + drone width
             (horizontal, so balloons are never stacked)
  compact    every balloon within --max-radius of the take-off point, and
             every balloon has a neighbour within --max-neighbour-distance
             (the field is one connected cluster, not scattered)
  take-off   no balloon within --min-takeoff-distance of the take-off pad
  field      inside the 10 m x 10 m boundary with --boundary-margin
  heights    uniform in [--min-height, --max-height] (balloon centres)

Balloon counts per colour default to those of the base file.

Usage:
  generate_arena.py --seed 42 [--base arena_default.yaml] [--out arena.yaml]
                    [--count red=2 --count green=2 ...] [constraint options]
Prints the arena YAML to stdout if --out is not given. Exit code 1 if no
layout satisfying the constraints was found.
"""

import argparse
import math
import os
import random
import sys

import yaml

FIELD_HALF_M = 5.0  # balloon_arena.sdf.xacro field boundary


def default_base() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in (os.path.join(here, '..', 'config', 'arena_default.yaml'),
                      os.path.join(here, '..', '..', 'share', 'pluto_x_gazebo',
                                   'config', 'arena_default.yaml')):
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    raise FileNotFoundError('arena_default.yaml not found; pass --base')


def horizontal(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def check_layout(positions, p):
    """Returns a list of violated constraints (empty = valid)."""
    problems = []
    min_centre = p['diameter_m'] + p['drone_width_m']
    for i, a in enumerate(positions):
        r = math.hypot(a[0], a[1])
        if r > p['max_radius_m'] + 1e-9:
            problems.append(f'balloon {i} at {r:.2f} m > max radius')
        if r < p['min_takeoff_distance_m'] - 1e-9:
            problems.append(f'balloon {i} at {r:.2f} m < take-off clearance')
        limit = FIELD_HALF_M - p['boundary_margin_m']
        if abs(a[0]) > limit or abs(a[1]) > limit:
            problems.append(f'balloon {i} outside the field')
        if not p['min_height_m'] - 1e-9 <= a[2] <= p['max_height_m'] + 1e-9:
            problems.append(f'balloon {i} height {a[2]:.2f} m out of range')
        for j in range(i + 1, len(positions)):
            d = horizontal(a, positions[j])
            if d < min_centre - 1e-9:
                problems.append(f'balloons {i},{j}: centre distance {d:.2f} m < {min_centre:.2f} m')
    if len(positions) > 1:
        # connectivity: every balloon reachable through neighbours within max_neighbour
        reached, frontier = {0}, [0]
        while frontier:
            i = frontier.pop()
            for j in range(len(positions)):
                if j not in reached and horizontal(positions[i], positions[j]) \
                        <= p['max_neighbour_distance_m'] + 1e-9:
                    reached.add(j)
                    frontier.append(j)
        if len(reached) != len(positions):
            problems.append('balloons do not form one cluster '
                            f'(neighbour distance > {p["max_neighbour_distance_m"]} m)')
    return problems


def generate(counts, p, seed, attempts=20000):
    rng = random.Random(seed)
    total = sum(counts.values())
    min_centre = p['diameter_m'] + p['drone_width_m']
    for _ in range(attempts):
        positions = []
        for _k in range(total):
            for _try in range(500):
                # uniform in the annulus [min_takeoff, max_radius]
                r = math.sqrt(rng.uniform(p['min_takeoff_distance_m'] ** 2,
                                          p['max_radius_m'] ** 2))
                theta = rng.uniform(-math.pi, math.pi)
                c = (r * math.cos(theta), r * math.sin(theta),
                     rng.uniform(p['min_height_m'], p['max_height_m']))
                limit = FIELD_HALF_M - p['boundary_margin_m']
                if abs(c[0]) > limit or abs(c[1]) > limit:
                    continue
                if all(horizontal(c, q) >= min_centre for q in positions) and \
                        (not positions or min(horizontal(c, q) for q in positions)
                         <= p['max_neighbour_distance_m']):
                    positions.append(c)
                    break
            else:
                break
        # check the positions as written (rounded to 1 mm), not before rounding
        positions = [[round(v, 3) for v in pos] for pos in positions]
        if len(positions) == total and not check_layout(positions, p):
            colours = [c for c, n in counts.items() for _ in range(n)]
            rng.shuffle(colours)
            return [{'color': col, 'position_enu_m': pos}
                    for col, pos in zip(colours, positions)]
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--base', default='')
    parser.add_argument('--out', default='')
    parser.add_argument('--count', action='append', default=[],
                        help='colour=N (default: counts of the base file)')
    parser.add_argument('--drone-width', type=float, default=0.16,
                        help='min gap between balloon surfaces, m (Pluto X: 16 cm)')
    parser.add_argument('--max-radius', type=float, default=3.5)
    parser.add_argument('--max-neighbour-distance', type=float, default=2.0)
    parser.add_argument('--min-takeoff-distance', type=float, default=1.0)
    parser.add_argument('--min-height', type=float, default=0.8)
    parser.add_argument('--max-height', type=float, default=2.0)
    parser.add_argument('--boundary-margin', type=float, default=0.5)
    args = parser.parse_args(argv)

    base_path = args.base or default_base()
    with open(base_path, 'r', encoding='utf-8') as stream:
        arena = yaml.safe_load(stream)
    counts = {}
    for b in arena['balloons']:
        counts[b['color']] = counts.get(b['color'], 0) + 1
    for item in args.count:
        colour, _, n = item.partition('=')
        if colour not in arena['colors']:
            parser.error(f'unknown colour {colour!r} (known: {sorted(arena["colors"])})')
        counts[colour] = int(n)
    params = {
        'diameter_m': float(arena['balloon']['diameter_m']),
        'drone_width_m': args.drone_width,
        'max_radius_m': args.max_radius,
        'max_neighbour_distance_m': args.max_neighbour_distance,
        'min_takeoff_distance_m': args.min_takeoff_distance,
        'min_height_m': args.min_height,
        'max_height_m': args.max_height,
        'boundary_margin_m': args.boundary_margin,
    }
    balloons = generate(counts, params, args.seed)
    if balloons is None:
        print('no layout satisfies the constraints; relax them', file=sys.stderr)
        return 1
    arena['balloons'] = balloons
    header = (f'# Generated by generate_arena.py --seed {args.seed} from {os.path.basename(base_path)}\n'
              f'# constraints: {params}\n')
    text = header + yaml.safe_dump(arena, sort_keys=False, default_flow_style=None)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as out:
            out.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
