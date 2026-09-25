#!/usr/bin/env python3
"""Evaluates an outer-loop controller over many random balloon layouts.

    python3 evaluation/evaluate.py --controller outerloop_controller/my_controller.py --runs 10

Each run is a normal competition run (headless, 15 s from arming), with its
own random layout. Everything is saved to
evaluation/sessions/<date-time>/:

  session.yaml          controller file + SHA-256, seeds, date, the summary
  summary.csv           one row per run (score, balloons per colour, red
                        hits, time to the first pop, status, ...)
  run_XX_seed_N/        the full recording of each run (same files as
                        analysis/runs/<run>/: trajectory, telemetry,
                        commands, events, camera video, ...)
  logs/run_XX.log       the simulator + controller output of each run

Then open evaluation/evaluation.ipynb to see the results.

Options:
  --runs N            number of runs (default 10)
  --seeds 5,17,301    use these seeds (overrides --runs); for judging, the
                      organisers use a list of seeds the teams have not seen
  --timeout S         wall-clock limit per run (default 120 s)

Runs are sequential (each renders a 720p camera); about 35 s per run.
Needs the workspace sourced (source install/setup.bash). summary.csv and
session.yaml are rewritten after every run, so an interrupted session keeps
the runs done so far.

Scores: runs where the simulator failed ('crashed', 'timeout' with the
drone armed) are reported separately; a run where the controller never
armed counts as a 0-point run (mean_score_all), as it would in judging.
"""

import argparse
import atexit
import csv
import hashlib
import os
import random
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import time
from datetime import datetime

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SESSIONS = os.path.join(HERE, 'sessions')
sys.path.insert(0, os.path.join(REPO, 'analysis'))
import runlog  # noqa: E402

SHUTDOWN_GRACE_S = 10.0
_ACTIVE = set()    # process groups of running launches (killed on any exit)


def sha256(path):
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def analysis_seed():
    try:
        with open(os.path.join(REPO, 'analysis', 'seed.yaml'), encoding='utf-8') as stream:
            return int(yaml.safe_load(stream)['seed'])
    except (OSError, KeyError, TypeError, ValueError):
        return None


def _killpg(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def stop(process):
    """Stops a launch and EVERYTHING it started (its own process group): a
    polite SIGINT first, then SIGKILL for whatever is left (gz sim can
    outlive ros2 launch). A second Ctrl-C during the wait skips to SIGKILL."""
    try:
        if process.poll() is None:
            _killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=SHUTDOWN_GRACE_S)
            except subprocess.TimeoutExpired:
                pass
    finally:
        _killpg(process.pid, signal.SIGKILL)   # always: leftovers in the group
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass
        _ACTIVE.discard(process.pid)


@atexit.register
def _kill_active():
    for pgid in list(_ACTIVE):
        _killpg(pgid, signal.SIGKILL)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt  # SIGTERM / SIGHUP: the same clean-up as Ctrl-C


def run_one(index, seed, controller, session, timeout_s):
    name = f'run_{index:02d}_seed_{seed}'
    run_dir = os.path.join(session, name)
    log_path = os.path.join(session, 'logs', f'run_{index:02d}.log')
    env = dict(os.environ)
    # isolate each run from anything else (other runs, a simulator you have
    # open, other laptops on the same Wi-Fi)
    env['GZ_PARTITION'] = f'avatic_eval_{socket.gethostname()}_{os.getpid()}_{index}'
    env['GZ_IP'] = '127.0.0.1'
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['ROS_DOMAIN_ID'] = str(1 + (os.getpid() + index) % 100)
    command = ['ros2', 'launch', 'pluto_x_bringup', 'competition.launch.py',
               'headless:=true', 'rviz:=false', f'controller:={controller}',
               f'arena_seed:={seed}', f'record_dir:={session}', f'run_name:={name}']
    started = time.monotonic()
    with open(log_path, 'w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                   start_new_session=True)
        _ACTIVE.add(process.pid)
        status = 'timeout'
        try:
            while time.monotonic() - started < timeout_s:
                # the recorder writes result.yaml last: all files are complete
                if os.path.isfile(os.path.join(run_dir, 'result.yaml')):
                    status = 'complete'
                    break
                if process.poll() is not None:
                    status = 'crashed'
                    break
                time.sleep(0.5)
        finally:
            stop(process)
    return name, run_dir, status, time.monotonic() - started


def row_for(name, run_dir, seed, status, wall_s, controller=None):
    row = {'run': name, 'seed': seed, 'status': status, 'wall_time_s': round(wall_s, 1)}
    if controller:   # per run: the file may be edited during a session
        row['controller_sha256'] = sha256(controller)[:16]
    try:
        _fill_row(row, run_dir)
    except Exception as error:  # noqa: BLE001 - one damaged run must not end the session
        row['error'] = f'{type(error).__name__}: {error}'
    # the flight controller was ready but the controller never armed: its
    # failure (0 points). Not ready, or no recording: the simulator's.
    if row['status'] == 'timeout' and row.get('ready') and not row.get('armed'):
        row['status'] = 'never_armed'
    return row


def _fill_row(row, run_dir):
    row['controller_error'] = int(_controller_raised(run_dir))
    run = runlog.load_run(os.path.basename(run_dir), root=os.path.dirname(run_dir))
    row['armed'] = int(run.arm_time_s is not None)
    ready = run.telemetry.get('ready_to_arm')
    row['ready'] = int(ready is not None and bool((ready > 0.5).any()))
    s = runlog.summary(run)
    row.update({'score': s.get('score'), 'max_score': s.get('max_score'),
                'red_hits': s.get('red_hits'),
                'first_pop_after_arm_s': s.get('first_pop_after_arm_s'),
                'max_height_m': s.get('max_height_m'), 'max_tilt_deg': s.get('max_tilt_deg'),
                'distance_flown_m': s.get('distance_flown_m')})
    for colour in runlog.BALLOON_COLOURS:
        here = [b for b in run.balloons if b['colour'] == colour]
        row[f'{colour}_popped'] = sum(1 for b in here if b['popped'])
        row[f'{colour}_total'] = len(here)


def _controller_raised(run_dir):
    log = os.path.join(os.path.dirname(run_dir), 'logs',
                       f"run_{os.path.basename(run_dir).split('_')[1]}.log")
    try:
        with open(log, encoding='utf-8', errors='replace') as stream:
            return 'step() raised an error' in stream.read()
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controller', default='outerloop_controller/my_controller.py')
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--seeds', default='', help='comma-separated seeds (overrides --runs)')
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args()

    controller = os.path.abspath(args.controller)
    if not os.path.isfile(controller):
        controller = os.path.join(REPO, args.controller)
    if not os.path.isfile(controller):
        parser.error(f'controller not found: {args.controller}')
    if args.seeds:
        seeds = [int(s) for s in args.seeds.split(',') if s.strip()]
    else:
        rng, avoid, seeds = random.SystemRandom(), analysis_seed(), []
        while len(seeds) < args.runs:
            s = rng.randrange(1, 1_000_000)
            if s != avoid and s not in seeds:   # never the development layout
                seeds.append(s)
    if shutil.which('ros2') is None:
        parser.error('ros2 not found: source /opt/ros/humble/setup.bash && '
                     'source install/setup.bash first')
    stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    session = os.path.join(SESSIONS, stamp)
    suffix = 1
    while os.path.exists(session):          # two sessions started in the same second
        suffix += 1
        session = os.path.join(SESSIONS, f'{stamp}_{suffix}')
    os.makedirs(os.path.join(session, 'logs'))
    meta = {'controller': controller, 'controller_sha256': sha256(controller),
            'started_local_time': datetime.now().isoformat(timespec='seconds'),
            'seeds': seeds, 'runs': len(seeds)}
    print(f'evaluation session {session}\n  controller {controller}\n  {len(seeds)} runs, '
          f'seeds {seeds}\n  about {len(seeds) * 35 // 60 + 1} min', flush=True)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    signal.signal(signal.SIGHUP, _raise_interrupt)

    rows = []
    try:
        for i, seed in enumerate(seeds, start=1):
            name, run_dir, status, wall = run_one(i, seed, controller, session, args.timeout)
            row = row_for(name, run_dir, seed, status, wall, controller)
            rows.append(row)
            save(session, meta, rows)             # after every run
            print(f"  [{i:2d}/{len(seeds)}] seed {seed:>6}: {row['status']:<11} score "
                  f"{row.get('score')}  (green {row.get('green_popped', 0)}, blue "
                  f"{row.get('blue_popped', 0)}, yellow {row.get('yellow_popped', 0)}, red "
                  f"{row.get('red_popped', 0)})  {wall:4.0f} s", flush=True)
    except KeyboardInterrupt:
        print('\ninterrupted: saving the runs done so far', flush=True)
    finally:
        _kill_active()
        save(session, meta, rows, finished=True)
    print(f"\nRESULT: mean score {meta['mean_score']} over {meta['completed_runs']} completed "
          f"runs (min {meta['min_score']}, max {meta['best_run_score']}; best possible "
          f"{meta['best_possible_per_run']} per run); mean incl. never-armed runs as 0: "
          f"{meta['mean_score_all']}; red hits {meta['red_hits_total']}; simulator failures "
          f"{meta['simulator_failures']}\nsaved to {session}\n"
          'open evaluation/evaluation.ipynb to see the results', flush=True)


def save(session, meta, rows, finished=False):
    """Writes summary.csv and session.yaml (atomically: safe to interrupt)."""
    keys = []
    for row in rows:
        keys += [k for k in row if k not in keys]
    tmp = os.path.join(session, 'summary.csv.tmp')
    with open(tmp, 'w', newline='', encoding='utf-8') as out:
        writer = csv.DictWriter(out, fieldnames=keys or ['run'])
        writer.writeheader()
        writer.writerows(rows)
    os.replace(tmp, os.path.join(session, 'summary.csv'))
    complete = [r for r in rows if r.get('status') == 'complete' and r.get('score') is not None]
    scores = [r['score'] for r in complete]
    # judged: a controller that never armed or raised still scores 0
    scores_all = scores + [0 for r in rows if r.get('status') == 'never_armed']
    meta.update({
        'finished_local_time': datetime.now().isoformat(timespec='seconds') if finished else None,
        'runs_done': len(rows),
        'completed_runs': len(complete),
        'never_armed_runs': sum(1 for r in rows if r.get('status') == 'never_armed'),
        'simulator_failures': sum(1 for r in rows if r.get('status') in ('crashed', 'timeout')),
        'mean_score': round(statistics.mean(scores), 1) if scores else None,
        'mean_score_all': round(statistics.mean(scores_all), 1) if scores_all else None,
        'min_score': min(scores) if scores else None,
        'best_run_score': max(scores) if scores else None,
        'best_possible_per_run': next((r['max_score'] for r in rows
                                       if r.get('max_score') is not None), None),
        'red_hits_total': sum(r.get('red_popped') or 0 for r in complete),
    })
    tmp = os.path.join(session, 'session.yaml.tmp')
    with open(tmp, 'w', encoding='utf-8') as out:
        yaml.safe_dump(meta, out, sort_keys=False)
    os.replace(tmp, os.path.join(session, 'session.yaml'))


if __name__ == '__main__':
    main()
