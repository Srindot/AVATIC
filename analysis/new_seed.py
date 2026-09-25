#!/usr/bin/env python3
"""Sets the balloon-layout seed used by development runs (analysis/seed.yaml).

    python3 analysis/new_seed.py            # pick a new random layout
    python3 analysis/new_seed.py --seed 7   # use seed 7
    python3 analysis/new_seed.py --show     # print the current seed
"""
import argparse
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
SEED_FILE = os.path.join(HERE, 'seed.yaml')
HEADER = ('# Balloon-layout seed for development runs (see analysis/README.md).\n'
          '# Every competition launch uses it until you change it:\n'
          '#   python3 analysis/new_seed.py          # a new random layout\n'
          '#   python3 analysis/new_seed.py --seed 7 # a specific layout\n')


def current():
    if not os.path.isfile(SEED_FILE):
        return None
    for line in open(SEED_FILE, encoding='utf-8'):
        if line.strip().startswith('seed:'):
            return int(line.split(':', 1)[1])
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seed', type=int, help='use this seed instead of a random one')
    parser.add_argument('--show', action='store_true', help='print the current seed and exit')
    args = parser.parse_args()
    if args.show:
        print(f'current seed: {current()}')
        return
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(1, 1_000_000)
    old = current()
    with open(SEED_FILE, 'w', encoding='utf-8') as out:
        out.write(HEADER + f'seed: {seed}\n')
    print(f'seed {old} -> {seed}: the next runs use a new balloon layout')


if __name__ == '__main__':
    main()
