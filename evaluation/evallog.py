"""Reads the sessions saved by evaluate.py (evaluation/sessions/<date-time>/)
and plots them. Used by evaluation.ipynb; NumPy + Matplotlib only.

    import evallog
    s = evallog.load_session()          # the latest session
    evallog.show_objective(s)           # the headline: mean score, per colour, red hits
    evallog.plot_scores(s)
    run = evallog.load_run(s, 'best')   # a runlog.Run: all the analysis plots work on it
"""

import csv
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SESSIONS_DIR = os.path.join(HERE, 'sessions')
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'analysis'))
import runlog  # noqa: E402

COLOURS = runlog.BALLOON_COLOURS


def _points():
    """Points per colour, from the arena config (the official rules)."""
    points = {'green': 100, 'blue': 50, 'yellow': 25, 'red': -300}
    colours = (runlog._load_yaml(runlog.RULES_FILE) or {}).get('colors', {})
    points.update({c: int(v['points']) for c, v in colours.items() if c in points})
    return points


POINTS = _points()


@dataclass
class Session:
    session_id: str
    path: str
    meta: dict
    rows: List[dict]            # summary.csv, one per run (numbers converted)

    @property
    def complete(self) -> List[dict]:
        return [r for r in self.rows if r.get('status') == 'complete' and r.get('score') is not None]

    def scores(self) -> np.ndarray:
        return np.array([r['score'] for r in self.complete], dtype=float)


def list_sessions(root: str = SESSIONS_DIR) -> List[str]:
    if not os.path.isdir(root):
        return []
    return sorted(d for d in os.listdir(root)
                  if os.path.isfile(os.path.join(root, d, 'summary.csv')))


def _number(value):
    if value in ('', None):
        return None
    try:
        f = float(value)
    except ValueError:
        return value
    return int(f) if f.is_integer() else f


def load_session(session_id: Optional[str] = None, root: str = SESSIONS_DIR) -> Session:
    """Loads a session; None = the latest one."""
    sessions = list_sessions(root)
    if not sessions:
        raise FileNotFoundError(f'no evaluation sessions in {root}: run evaluation/evaluate.py first')
    session_id = session_id or sessions[-1]
    path = os.path.join(root, session_id)
    if not os.path.isdir(path):
        raise FileNotFoundError(f'session {session_id!r} not found; available: {sessions[-10:]}')
    meta = {}
    if os.path.isfile(os.path.join(path, 'session.yaml')):
        with open(os.path.join(path, 'session.yaml'), encoding='utf-8') as stream:
            meta = yaml.safe_load(stream) or {}
    with open(os.path.join(path, 'summary.csv'), newline='', encoding='utf-8') as stream:
        text = ('run', 'status', 'error', 'controller_sha256')
        rows = [{k: (v if k in text else _number(v)) for k, v in row.items()}
                for row in csv.DictReader(stream)]
    return Session(session_id=session_id, path=path, meta=meta, rows=rows)


def _recorded(session: Session, row: dict) -> bool:
    return os.path.isfile(os.path.join(session.path, str(row.get('run')), 'meta.yaml'))


def load_run(session: Session, which='best') -> Optional[runlog.Run]:
    """One run of the session as a runlog.Run: 'best', 'worst', an index
    (1 = the first run) or a run name (run_03_seed_1234). None if there is
    no such recorded run (e.g. no run completed)."""
    rows = [r for r in (session.complete or session.rows) if _recorded(session, r)]
    score = (lambda r: r['score'] if r.get('score') is not None else None)
    if which in ('best', 'worst'):
        scored = [r for r in rows if score(r) is not None] or rows
        if not scored:
            return None
        pick = max if which == 'best' else min
        name = pick(scored, key=lambda r: score(r) if score(r) is not None else 0)['run']
    elif isinstance(which, int):
        if not 1 <= which <= len(session.rows) or not _recorded(session, session.rows[which - 1]):
            return None
        name = session.rows[which - 1]['run']
    else:
        name = which
    return runlog.load_run(name, root=session.path)


# ----------------------------------------------------------------- headline

def objective(session: Session) -> dict:
    scores = session.scores()
    rows = session.complete
    best = session.meta.get('best_possible_per_run') or next(
        (r['max_score'] for r in session.rows if r.get('max_score') is not None), None)
    never_armed = sum(1 for r in session.rows if r.get('status') == 'never_armed')
    scores_all = np.concatenate([scores, np.zeros(never_armed)])
    out = {'runs': len(session.rows), 'completed': len(rows), 'never_armed': never_armed,
           'failed': len(session.rows) - len(rows) - never_armed,   # simulator failures
           'best_possible_per_run': best,
           'mean_score': round(float(scores.mean()), 1) if len(scores) else None,
           'mean_score_all': round(float(scores_all.mean()), 1) if len(scores_all) else None,
           'median_score': float(np.median(scores)) if len(scores) else None,
           'min_score': int(scores.min()) if len(scores) else None,
           'max_score': int(scores.max()) if len(scores) else None,
           # sample standard deviation (over the runs, not a full population)
           'std_score': round(float(scores.std(ddof=1)), 1) if len(scores) > 1 else None,
           'red_hits_total': sum(r.get('red_popped') or 0 for r in rows),
           'runs_with_red_hit': sum(1 for r in rows if (r.get('red_popped') or 0) > 0),
           'runs_scoring_zero_or_less': int(np.sum(scores <= 0)) if len(scores) else 0,
           'controller_errors': sum(r.get('controller_error') or 0 for r in session.rows),
           # rows from before the check existed have no 'official_rules': unknown
           'not_official': sum(1 for r in session.rows if r.get('official_rules') == 0),
           'official_unknown': sum(1 for r in session.rows if r.get('official_rules') is None
                                   and r.get('status') == 'complete'),
           'fair_play_flagged': [r['run'] for r in session.rows if r.get('fair_play') == 'flagged'],
           'fair_play_unchecked': sum(1 for r in session.rows
                                      if r.get('fair_play') in (None, 'not checked')
                                      and r.get('status') == 'complete'),
           'code_check': session.meta.get('code_check'),
           'colours': {}}
    for colour in COLOURS:
        popped = sum(r.get(f'{colour}_popped') or 0 for r in rows)
        # per-run totals from each layout (sessions from before this was recorded: 2 per colour)
        available = sum(r.get(f'{colour}_total', 2) or 0 for r in rows)
        out['colours'][colour] = {'popped': popped, 'available': available}
    return out


def _rules_md(o: dict) -> str:
    rules = runlog.official_rules()
    counts = rules.get('balloon_counts', {})
    desc = (f"{rules.get('time_limit_s', '?')} s from arming, {sum(counts.values())} balloons "
            f"(green {counts.get('green', '?')}, blue {counts.get('blue', '?')}, "
            f"yellow {counts.get('yellow', '?')}, red {counts.get('red', '?')})")
    if o['not_official']:
        return (f"> **NOT AN OFFICIAL EVALUATION:** {o['not_official']} run(s) used other rules "
                f"than the official ones ({desc}). Results with changed rules are not "
                'comparable and do not count.')
    if o['official_unknown']:
        return f'**Rules:** official rules: {desc} (not recorded for {o["official_unknown"]} older run(s)).'
    return f'**Rules:** every run used the official competition rules: {desc}.'


def _fair_play_md(o: dict) -> str:
    code = o['code_check']
    code_text = ('code check: not recorded' if code is None else
                 'code check: clean' if code.get('verdict') == 'clean' else
                 f"code check: {len(code.get('findings', []))} finding(s) to review "
                 "(session.yaml, code_check)")
    if o['fair_play_flagged']:
        return (f"> **FLAGGED BY THE FAIR-PLAY CHECK:** {len(o['fair_play_flagged'])} run(s) "
                f"({', '.join(o['fair_play_flagged'])}; see the fair_play_flags column); "
                f"{code_text}. An organiser reviews the controller before these results count.")
    if code and code.get('verdict') != 'clean':
        return f'> **Fair play:** no run flagged; {code_text}.'
    unchecked = (f" (not recorded for {o['fair_play_unchecked']} older run(s))"
                 if o['fair_play_unchecked'] else '')
    return f'**Fair play:** no run flagged{unchecked}; {code_text}.'


def show_objective(session: Session) -> None:
    o = objective(session)
    fmt = (lambda v: '?' if v is None else v)
    rate = (lambda c: f"{100 * c['popped'] / c['available']:.0f} %" if c['available'] else '-')
    # the headline is the judged-style mean: a run that never armed scores 0
    md = [f"## Mean score: **{fmt(o['mean_score_all'])}** / {fmt(o['best_possible_per_run'])} per run",
          '',
          f"session `{session.session_id}`: {o['completed']} of {o['runs']} runs completed"
          + (f" (**{o['failed']} simulator failures**)" if o['failed'] else '')
          + (f" (**{o['never_armed']} never armed**, counted as 0)" if o['never_armed'] else ''),
          f"controller `{os.path.basename(session.meta.get('controller', '?'))}` "
          f"(sha256 {str(session.meta.get('controller_sha256', '?'))[:12]})", '',
          _rules_md(o), '', _fair_play_md(o), '',
          '| | |', '|---|---|',
          f"| min / median / max | {fmt(o['min_score'])} / {fmt(o['median_score'])} / {fmt(o['max_score'])} |",
          f"| mean over completed runs only | {fmt(o['mean_score'])} |",
          f"| standard deviation | {fmt(o['std_score'])} |",
          f"| runs scoring 0 or less | {o['runs_scoring_zero_or_less']} |",
          f"| **red balloons hit** | **{o['red_hits_total']}** (in {o['runs_with_red_hit']} runs) |",
          f"| controller errors (step() raised) | {o['controller_errors']} |", '',
          '| balloon | popped (all runs) | hit rate |', '|---|---|---|']
    md += [f"| {c} | {v['popped']} / {v['available']} | {rate(v)} |" for c, v in o['colours'].items()]
    text = [_rules_md(o).replace('**', '').lstrip('> '),
            _fair_play_md(o).replace('**', '').lstrip('> '),
            f"MEAN SCORE {fmt(o['mean_score_all'])} / {fmt(o['best_possible_per_run'])} per run "
            f"({o['completed']}/{o['runs']} runs completed, {o['failed']} failed)",
            f"  min {fmt(o['min_score'])}  median {fmt(o['median_score'])}  max {fmt(o['max_score'])}"
            f"  std {fmt(o['std_score'])}",
            f"  red hits {o['red_hits_total']} (in {o['runs_with_red_hit']} runs); "
            f"controller errors {o['controller_errors']}"]
    text += [f"  {c:<7} popped {v['popped']:>3} / {v['available']:<3} {rate(v)}"
             for c, v in o['colours'].items()]
    runlog._show('\n'.join(md), '\n'.join(text))


def table(session: Session) -> None:
    """Per-run table."""
    cols = ['run', 'status', 'score', 'green_popped', 'blue_popped', 'yellow_popped', 'red_popped',
            'first_pop_after_arm_s', 'distance_flown_m', 'max_tilt_deg', 'controller_error']
    head = ['run', 'status', 'score', 'green', 'blue', 'yellow', 'red', 'first pop (s)',
            'distance (m)', 'max tilt (deg)', 'error']
    md = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    text = []
    for r in session.rows:
        cells = ['' if r.get(c) is None else str(r.get(c)) for c in cols]
        md.append('| ' + ' | '.join(cells) + ' |')
        text.append('  '.join(f'{c:>8}' for c in cells))
    runlog._show('\n'.join(md), '\n'.join(text))


# ----------------------------------------------------------------- plots

def plot_scores(session: Session):
    """Points per run, split by colour: the good balloons stack up from 0,
    red hits hang below it; the black dash is the run's score."""
    plt = runlog._plt()
    rows = session.rows
    fig, ax = plt.subplots(figsize=(max(8.0, 0.45 * len(rows) + 2), 4.5))
    x = np.arange(len(rows))
    bottom = np.zeros(len(rows))
    for colour in COLOURS[:3]:
        points = np.array([(r.get(f'{colour}_popped') or 0) * POINTS[colour] for r in rows],
                          dtype=float)
        ax.bar(x, points, bottom=bottom, color=runlog.COLOURS[colour], label=colour)
        bottom += points
    red = np.array([(r.get('red_popped') or 0) * POINTS['red'] for r in rows], dtype=float)
    ax.bar(x, red, color=runlog.COLOURS['red'], label='red (penalty)')
    for i, r in enumerate(rows):
        if r.get('status') == 'complete' and r.get('score') is not None:
            ax.plot([i - 0.4, i + 0.4], [r['score']] * 2, color='k', lw=2)
        else:
            ax.text(i, 5, r.get('status', '?').replace('_', ' '), ha='center', va='bottom',
                    rotation=90, fontsize=8, color='0.3')
    o = objective(session)
    if o['mean_score_all'] is not None:
        ax.axhline(o['mean_score_all'], color='k', ls='--', lw=1,
                   label=f"mean score {o['mean_score_all']}")
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xticks(x)
    ax.set_xticklabels([str(i + 1) for i in x])
    ax.set_xlabel('run')
    ax.set_ylabel('points')
    best = o['best_possible_per_run']
    ax.set_title('points per run by colour; black dash = the run\'s score'
                 + (f' (best possible {best})' if best else ''))
    top = max([*bottom, *(r.get('score') or 0 for r in rows), 1.0])
    ax.set_ylim(min(red.min(initial=0.0), 0.0) * 1.15 - 5, top * 1.12)
    ax.legend(fontsize=8, ncol=5, loc='upper center', bbox_to_anchor=(0.5, -0.15))
    ax.grid(alpha=0.3, axis='y')
    fig.tight_layout()
    return fig


def plot_score_over_time(session: Session):
    """The score of every run against the time since arming (thin lines) and
    their mean (thick): how fast points come, and whether they stop coming."""
    plt = runlog._plt()
    fig, ax = plt.subplots(figsize=(9, 4))
    limit = None
    grid, curves = None, []
    for row in session.rows:
        if not _recorded(session, row) or row.get('status') != 'complete':
            continue
        run = runlog.load_run(row['run'], root=session.path)
        t_arm = run.arm_time_s
        if t_arm is None:
            continue
        limit = limit or run.meta.get('time_limit_s')
        ev = run.events
        times, totals = [0.0], [0.0]
        for t, kind, total in zip(ev.get('t_s', []), ev.get('kind', []), ev.get('total', [])):
            if kind == 'pop':
                times.append(float(t) - t_arm)
                totals.append(float(total))
        end = float(limit or max(times))
        times.append(end)
        totals.append(totals[-1])
        ax.step(times, totals, where='post', color='tab:blue', lw=0.8, alpha=0.35)
        grid = np.linspace(0, end, 251) if grid is None else grid
        curves.append(np.array(totals)[np.searchsorted(times, grid, side='right') - 1])
    if curves:
        ax.plot(grid, np.mean(curves, axis=0), color='k', lw=2.5,
                label=f'mean of {len(curves)} runs')
        ax.legend(fontsize=8, loc='upper left')
    else:
        ax.set_title('no recorded runs')
    if limit:
        ax.axvline(limit, color='tab:red', ls=':', lw=1)
    ax.axhline(0, color='k', lw=0.5)
    ax.set_xlabel('s after arming')
    ax.set_ylabel('score so far')
    ax.set_title('score during the run (each thin line = one run)')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return fig


def plot_colours(session: Session):
    """Balloons popped per colour, summed over the runs."""
    plt = runlog._plt()
    o = objective(session)
    fig, ax = plt.subplots(figsize=(6, 3.5))
    cs = list(o['colours'])
    popped = [o['colours'][c]['popped'] for c in cs]
    available = [o['colours'][c]['available'] for c in cs]
    ax.bar(cs, available, color='none', edgecolor='k', ls='--', label='available')
    ax.bar(cs, popped, color=['tab:green', 'tab:blue', 'gold', 'tab:red'], label='popped')
    ax.set_ylabel('balloons (all runs)')
    ax.set_title('balloons popped per colour (red = penalty)')
    ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


def plot_first_pop(session: Session):
    """Time from arming to the first pop, per run (how fast the controller finds one)."""
    plt = runlog._plt()
    fig, ax = plt.subplots(figsize=(6, 3.5))
    t = [r.get('first_pop_after_arm_s') for r in session.rows]
    names = [str(i + 1) for i in range(len(t))]
    ax.bar(names, [v if v is not None else 0 for v in t],
           color=['tab:blue' if v is not None else 'tab:gray' for v in t])
    for i, v in enumerate(t):
        if v is None:
            ax.text(i, 0.2, 'none', ha='center', fontsize=8, rotation=90)
    limit = None
    try:
        run = load_run(session, 'best')
        limit = run.meta.get('time_limit_s') if run else None
    except (FileNotFoundError, OSError, ValueError, KeyError, IndexError):
        pass
    if limit:
        ax.axhline(limit, color='tab:red', ls=':', lw=1, label=f'time limit {limit} s')
        ax.legend(fontsize=8)
    ax.set_xlabel('run')
    ax.set_ylabel('s after arming')
    ax.set_title('time to the first pop')
    fig.tight_layout()
    return fig


def plot_maps(session: Session, runs=('best', 'worst')):
    """Top-view maps of some runs (default: the best and the worst)."""
    plt = runlog._plt()
    fig, axes = plt.subplots(1, len(runs), figsize=(6 * len(runs), 5.5), squeeze=False)
    for ax, which in zip(axes[0], runs):
        run = load_run(session, which)
        if run is None:
            ax.set_title(f'{which}: no recorded run')
            ax.axis('off')
            continue
        runlog.plot_map(run, ax=ax)
        score = (run.result or {}).get('score')
        ax.set_title(f'{which}: {run.run_id} (score {score})')
    fig.tight_layout()
    return fig
