#!/usr/bin/env python3
"""Collects your submission into this folder (output/). See DELIVERABLES.md.

    python3 output/collect.py                     # best recorded run, latest evaluation
    python3 output/collect.py --run 2026-09-25_14-32-07 --session 2026-09-26_10-00-00

It fills in everything that can be collected automatically:

  output/code/              your controller: outerloop_controller/ without
                            avatic_drone/, examples/ and README.md
  output/best_run/          the recording of your best run (analysis/runs/<run>/)
  output/analysis.ipynb     the analysis notebook, executed on that run
  output/evaluation/        session.yaml and summary.csv of your evaluation
  output/evaluation.ipynb   the evaluation notebook, executed on that session
  output/code_check.txt     the fair-play code check of your controller

and checks what you add yourself: output/video.mp4 (the screen recording of
the best run) and output/report.pdf (the technical report). Run it again
after any change: it replaces what it collected and never touches your
video or report. Needs Jupyter (the dev container has it); no simulator.
"""

import argparse
import glob
import hashlib
import os
import shutil
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
RUNS = os.path.join(REPO, 'analysis', 'runs')
SESSIONS = os.path.join(REPO, 'evaluation', 'sessions')
CONTROLLER_DIR = os.path.join(REPO, 'outerloop_controller')
NOT_YOUR_CODE = {'avatic_drone', 'examples', 'README.md', '__pycache__'}
MIN_EVALUATION_RUNS = 5
VIDEO_TYPES = ('.mp4', '.mkv', '.webm', '.mov')

sys.path[:0] = [os.path.join(REPO, 'analysis'), os.path.join(REPO, 'evaluation')]
import check_controller  # noqa: E402
import runlog  # noqa: E402


def _yaml(path):
    try:
        with open(path, encoding='utf-8') as stream:
            return yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        return {}


def _sha256(path):
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def best_run():
    """The highest-scoring complete run with the official rules, or None."""
    best = None
    for run_id in runlog.list_runs():
        try:
            run = runlog.load_run(run_id)
        except (OSError, ValueError, KeyError):
            continue
        score = (run.result or {}).get('score')
        if run.meta.get('status') != 'complete' or score is None:
            continue
        if not runlog.rules_check(run)['official'] or \
                runlog.fair_play(run)['verdict'] == 'flagged':
            continue
        if best is None or score > best[1]:
            best = (run_id, score)
    return best[0] if best else None


def latest_session():
    sessions = sorted(d for d in os.listdir(SESSIONS)
                      if os.path.isfile(os.path.join(SESSIONS, d, 'session.yaml'))) \
        if os.path.isdir(SESSIONS) else []
    return sessions[-1] if sessions else None


def execute_notebook(source, target, variable, value):
    """Runs a copy of the notebook with `variable = value` in its first code
    cell (from the notebook's own folder) and saves it, outputs included."""
    import nbformat
    from nbclient import NotebookClient
    nb = nbformat.read(source, as_version=4)
    first = next(c for c in nb.cells if c.cell_type == 'code')
    lines = first.source.split('\n')
    for i, line in enumerate(lines):
        if line.startswith(f'{variable} ='):
            lines[i] = f'{variable} = {value!r}'
            break
    first.source = '\n'.join(lines)
    NotebookClient(nb, timeout=600, resources={'metadata': {'path': os.path.dirname(source)}}
                   ).execute()
    nbformat.write(nb, target)


def copy_code(target):
    shutil.rmtree(target, ignore_errors=True)
    os.makedirs(target)
    for name in sorted(os.listdir(CONTROLLER_DIR)):
        if name in NOT_YOUR_CODE:
            continue
        path = os.path.join(CONTROLLER_DIR, name)
        if os.path.isdir(path):
            shutil.copytree(path, os.path.join(target, name),
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        else:
            shutil.copy2(path, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--run', help='the run for best_run/ (default: your best official run)')
    parser.add_argument('--session', help='the evaluation session (default: the latest)')
    parser.add_argument('--controller', default=os.path.join(CONTROLLER_DIR, 'my_controller.py'))
    args = parser.parse_args()
    problems, notes = [], []

    # 1. the code, and its fair-play check
    copy_code(os.path.join(HERE, 'code'))
    findings = check_controller.check_controller(args.controller)
    with open(os.path.join(HERE, 'code_check.txt'), 'w', encoding='utf-8') as out:
        out.write(f'fair-play code check of {os.path.relpath(args.controller, REPO)}: '
                  f'{check_controller.verdict(findings)}\n')
        out.writelines(f'{f}\n' for f in findings)
    if findings:
        notes.append(f'code check: {len(findings)} finding(s) in code_check.txt; explain each '
                     'one in your report (DELIVERABLES.md)')
    print(f'code          -> output/code/ (check: {check_controller.verdict(findings)})')

    # 2. the best run, and the analysis notebook on it
    run_id = args.run or best_run()
    if run_id is None:
        problems.append('no complete official run in analysis/runs/: fly one '
                        '(ros2 launch pluto_x_bringup competition.launch.py controller:=...)')
    else:
        run = runlog.load_run(run_id)
        target = os.path.join(HERE, 'best_run')
        shutil.rmtree(target, ignore_errors=True)
        shutil.copytree(run.path, target)
        score = (run.result or {}).get('score')
        print(f'best run      -> output/best_run/ ({run_id}, score {score}, seed '
              f"{run.meta.get('seed')})")
        if not runlog.rules_check(run)['official']:
            problems.append(f'run {run_id} did not use the official rules')
        verdict = runlog.fair_play(run)['verdict']
        if verdict != 'clean':
            problems.append(f'run {run_id}: fair play {verdict}')
        execute_notebook(os.path.join(REPO, 'analysis', 'analysis.ipynb'),
                         os.path.join(HERE, 'analysis.ipynb'), 'RUN', run_id)
        print('analysis      -> output/analysis.ipynb (executed)')

    # 3. the evaluation, and its notebook
    session_id = args.session or latest_session()
    if session_id is None:
        problems.append('no evaluation in evaluation/sessions/: run python3 '
                        f'evaluation/evaluate.py --runs {MIN_EVALUATION_RUNS}')
    else:
        source = os.path.join(SESSIONS, session_id)
        target = os.path.join(HERE, 'evaluation')
        shutil.rmtree(target, ignore_errors=True)
        os.makedirs(target)
        for name in ('session.yaml', 'summary.csv'):
            if os.path.isfile(os.path.join(source, name)):
                shutil.copy2(os.path.join(source, name), target)
        meta = _yaml(os.path.join(source, 'session.yaml'))
        print(f"evaluation    -> output/evaluation/ ({session_id}: {meta.get('runs_done')} runs, "
              f"mean score {meta.get('mean_score_all')})")
        if (meta.get('runs_done') or 0) < MIN_EVALUATION_RUNS:
            problems.append(f'the evaluation has {meta.get("runs_done")} runs; at least '
                            f'{MIN_EVALUATION_RUNS} are required')
        if meta.get('controller_sha256') and os.path.isfile(args.controller) and \
                meta['controller_sha256'] != _sha256(args.controller):
            problems.append('the evaluation was of a different version of your controller: '
                            'evaluate your final controller again')
        if meta.get('fair_play_flagged_runs'):
            problems.append(f"{meta['fair_play_flagged_runs']} evaluation run(s) flagged by "
                            'the fair-play check')
        execute_notebook(os.path.join(REPO, 'evaluation', 'evaluation.ipynb'),
                         os.path.join(HERE, 'evaluation.ipynb'), 'SESSION', session_id)
        print('evaluation    -> output/evaluation.ipynb (executed)')

    # 4. what only you can add
    videos = [p for p in glob.glob(os.path.join(HERE, 'video.*'))
              if p.lower().endswith(VIDEO_TYPES)]
    if not videos:
        problems.append('missing: output/video.mp4 (screen recording of the best run)')
    if not os.path.isfile(os.path.join(HERE, 'report.pdf')):
        problems.append('missing: output/report.pdf (technical report)')

    size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(HERE) for f in fs)
    print(f'\noutput/ is {size / 1e6:.0f} MB')
    for note in notes:
        print(f'  note: {note}')
    if problems:
        print('NOT READY:')
        for problem in problems:
            print(f'  - {problem}')
        sys.exit(1)
    print('READY: every deliverable is in output/ (DELIVERABLES.md)')


if __name__ == '__main__':
    main()
