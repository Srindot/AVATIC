"""Loading and plotting recorded runs (analysis/runs/<run_id>/).

Plain Python: NumPy, matplotlib, PyYAML and the ffmpeg command (for camera
frames). No ROS. Used by analysis.ipynb and (through evaluation/evallog.py) by
evaluation.ipynb.

    import runlog
    run = runlog.load_run()          # the latest run; or load_run('2026-09-25_14-32-07')
    runlog.print_summary(run)
    runlog.plot_overview(run)
"""

from __future__ import annotations

import csv
import os
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(HERE, 'runs')
COLOURS = {'yellow': '#e3c21a', 'blue': '#1f4fe0', 'green': '#1a9e2e', 'red': '#d11a1a'}
BALLOON_DIAMETER_M = 0.30


# ----------------------------------------------------------------- loading

def list_runs(root: str = RUNS_DIR) -> List[str]:
    """Run ids (directory names), oldest first."""
    if not os.path.isdir(root):
        return []
    return sorted(d for d in os.listdir(root)
                  if os.path.isfile(os.path.join(root, d, 'meta.yaml')))


def _read_csv(path: str) -> Dict[str, np.ndarray]:
    """CSV -> {column: array}; numeric columns as float, others as str."""
    if not os.path.isfile(path):
        return {}
    with open(path, newline='', encoding='utf-8') as stream:
        rows = list(csv.reader(stream))
    if not rows:
        return {}
    header, body = rows[0], rows[1:]
    if body and len(body[-1]) != len(header):
        body = body[:-1]   # a last line cut off when the recorder was killed
    columns = {}
    for i, name in enumerate(header):
        values = [r[i] if i < len(r) else '' for r in body]
        numbers = [_float_or_none(v) for v in values]
        if all(n is not None for n, v in zip(numbers, values) if v != ''):
            columns[name] = np.array([np.nan if n is None else n for n in numbers], dtype=float)
        else:
            columns[name] = np.array(values, dtype=object)
    return columns


def _float_or_none(value: str) -> Optional[float]:
    try:
        return float(value)
    except ValueError:
        return None


def _load_yaml(path: str):
    """YAML file -> data; None if missing or damaged (a run killed mid-write)."""
    try:
        with open(path, encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    except (OSError, yaml.YAMLError):
        return None


@dataclass
class Run:
    run_id: str
    path: str
    meta: dict
    result: Optional[dict]
    balloons: List[dict]                 # colour, points, position (3,), name, popped
    trajectory: Dict[str, np.ndarray]    # ground truth
    telemetry: Dict[str, np.ndarray]     # what the controller saw
    commands: Dict[str, np.ndarray]      # what reached the drone
    events: Dict[str, np.ndarray]
    camera_frames: Dict[str, np.ndarray]
    extra: dict = field(default_factory=dict)

    @property
    def arm_time_s(self) -> Optional[float]:
        armed = self.telemetry.get('armed')
        if armed is None or not np.any(armed > 0.5):
            return None
        return float(self.telemetry['t_s'][np.argmax(armed > 0.5)])

    @property
    def end_time_s(self) -> Optional[float]:
        ev = self.events
        if ev and 'kind' in ev:
            ends = [t for t, k in zip(ev['t_s'], ev['kind']) if k == 'end']
            if ends:
                return float(ends[0])
        t = self.trajectory.get('t_s')
        return float(t[-1]) if t is not None and len(t) else None

    def pops(self) -> List[dict]:
        """Pop events with the vehicle position at that moment."""
        out = []
        ev = self.events
        if not ev or 'kind' not in ev:
            return out
        for t, kind, colour, points in zip(ev['t_s'], ev['kind'], ev['colour'], ev['points']):
            if kind != 'pop':
                continue
            try:
                points = int(float(points))
            except (TypeError, ValueError):
                continue   # damaged event row
            out.append({'t_s': float(t), 'colour': colour, 'points': points,
                        'position': self.position_at(float(t))})
        return out

    def position_at(self, t: float) -> np.ndarray:
        tr = self.trajectory
        if not tr or 't_s' not in tr or not len(tr['t_s']) or not np.isfinite(t):
            return np.full(3, np.nan)
        return np.array([np.interp(t, tr['t_s'], tr[k]) for k in ('x_enu_m', 'y_enu_m', 'z_enu_m')])


def load_run(run_id: Optional[str] = None, root: str = RUNS_DIR) -> Run:
    """Loads a run; run_id=None loads the latest one."""
    runs = list_runs(root)
    if not runs:
        raise FileNotFoundError(f'no runs in {root}: fly one first '
                                '(ros2 launch pluto_x_bringup competition.launch.py ...)')
    run_id = run_id or runs[-1]
    path = os.path.join(root, run_id)
    if not os.path.isdir(path):
        raise FileNotFoundError(f'run {run_id!r} not found; available: {runs[-10:]}')
    meta = _load_yaml(os.path.join(path, 'meta.yaml')) or {}
    # result.yaml (the recorder, written last) or the arena's own copy
    result = (_load_yaml(os.path.join(path, 'result.yaml')) or
              _load_yaml(os.path.join(path, 'result_arena.yaml')))
    events = _read_csv(os.path.join(path, 'events.csv'))
    balloons = []
    layout = _load_yaml(os.path.join(path, 'layout.yaml'))
    if layout:
        if result:
            popped = {b['name']: b['popped'] for b in result.get('balloons', [])}
        else:   # no result (run cut short): the pop events name the balloons
            popped = {}
            for kind, text in zip(events.get('kind', []), events.get('text', [])):
                if kind == 'pop' and '(' in str(text):
                    popped[str(text).split('(', 1)[1].split(',')[0].strip()] = True
        for i, b in enumerate(layout['balloons']):
            name = f"balloon_{i}_{b['color']}"
            balloons.append({'name': name, 'colour': b['color'],
                             'points': layout['colors'][b['color']]['points'],
                             'position': np.array(b['position_enu_m'], dtype=float),
                             'popped': bool(popped.get(name, False))})
    return Run(run_id=run_id, path=path, meta=meta, result=result, balloons=balloons,
               trajectory=_read_csv(os.path.join(path, 'trajectory.csv')),
               telemetry=_read_csv(os.path.join(path, 'telemetry.csv')),
               commands=_read_csv(os.path.join(path, 'commands.csv')),
               events=events,
               camera_frames=_read_csv(os.path.join(path, 'camera_frames.csv')),
               extra={'layout': layout or {}})


# ----------------------------------------------------------------- summary

def summary(run: Run) -> dict:
    """Key numbers of a run."""
    r = run.result or {}
    popped = [b for b in run.balloons if b['popped']]
    pops = run.pops()
    t_arm = run.arm_time_s
    tr = run.trajectory
    out = {
        'run': run.run_id, 'status': run.meta.get('status'), 'seed': run.meta.get('seed'),
        'controller': os.path.basename(run.meta.get('controller') or '') or '(own terminal)',
        'score': r.get('score'), 'max_score': r.get('max_score'),
        'popped': ', '.join(f"{b['colour']} ({b['points']:+d})" for b in popped) or 'none',
        'red_hits': sum(1 for b in popped if b['points'] < 0),
        'first_pop_after_arm_s': (round(pops[0]['t_s'] - t_arm, 2)
                                  if pops and t_arm is not None else None),
    }
    flying = tr['t_s'] >= t_arm if tr and 't_s' in tr and t_arm is not None else None
    if flying is not None and flying.any():
        out['max_height_m'] = round(float(np.max(tr['z_enu_m'][flying])), 2)
        speed = np.hypot(tr['vx_enu_m_s'], tr['vy_enu_m_s'])[flying]
        out['max_horizontal_speed_m_s'] = round(float(np.max(speed)), 2)
        d = np.hypot(np.diff(tr['x_enu_m'][flying]), np.diff(tr['y_enu_m'][flying]))
        out['distance_flown_m'] = round(float(np.sum(d)), 2)
        tilt = np.degrees(np.arccos(np.clip(np.cos(tr['roll_rad']) * np.cos(tr['pitch_rad']), -1, 1)))
        out['max_tilt_deg'] = round(float(np.max(tilt[flying])), 1)
    return out


BALLOON_COLOURS = ('green', 'blue', 'yellow', 'red')


def objective(run: Run) -> dict:
    """What the run is judged on: the score, and the balloons per colour."""
    r = run.result or {}
    pops = run.pops()
    t_arm = run.arm_time_s
    out = {'score': r.get('score'), 'max_score': r.get('max_score'),
           'status': run.meta.get('status'), 'seed': run.meta.get('seed'),
           'time_limit_s': run.meta.get('time_limit_s'), 'colours': {}}
    for colour in BALLOON_COLOURS:
        here = [b for b in run.balloons if b['colour'] == colour]
        popped = [b for b in here if b['popped']]
        out['colours'][colour] = {'popped': len(popped), 'total': len(here),
                                  'points_each': here[0]['points'] if here else None,
                                  'points': sum(b['points'] for b in popped)}
    out['red_hits'] = out['colours']['red']['popped']
    out['pop_times_after_arm_s'] = ([round(p['t_s'] - t_arm, 2) for p in pops]
                                    if t_arm is not None else [])
    return out


def _show(markdown: str, text: str) -> None:
    try:
        from IPython import get_ipython
        from IPython.display import Markdown, display
        if get_ipython() is not None:
            display(Markdown(markdown))
            return
    except ImportError:
        pass
    print(text)


def show_objective(run: Run) -> None:
    """The headline: score, and what was popped (notebook: a table)."""
    o = objective(run)
    score = '?' if o['score'] is None else o['score']
    best = '?' if o['max_score'] is None else o['max_score']
    if o['status'] == 'complete':
        warn = ''
    elif o['score'] is not None:
        warn = f"  (run status: {o['status']})"
    else:
        warn = f"  (run status: {o['status']} - no final result; balloons from the pop events)"
    rows = []
    for colour, c in o['colours'].items():
        each = '' if c['points_each'] is None else f"{c['points_each']:+d}"
        rows.append((colour, each, f"{c['popped']} / {c['total']}", f"{c['points']:+d}"))
    md = [f"## Score: **{score}** / {best}{warn}", '',
          f"run `{run.run_id}`, seed {o['seed']}, {o['time_limit_s']} s from arming", '',
          '| balloon | points each | popped | points |', '|---|---|---|---|']
    md += [f'| {c} | {e} | {p} | {pts} |' for c, e, p, pts in rows]
    md += ['', f"**Red balloons hit: {o['red_hits']}**" + ('' if o['red_hits'] else ' (good)'),
           f"pops at {o['pop_times_after_arm_s']} s after arming" if o['pop_times_after_arm_s']
           else 'no balloon popped']
    text = [f"SCORE {score} / {best}{warn}", f"run {run.run_id}, seed {o['seed']}"]
    text += [f'  {c:<7}{e:>5}  popped {p:<6} {pts:>5}' for c, e, p, pts in rows]
    text += [f"  red hits: {o['red_hits']}", f"  pop times after arming: {o['pop_times_after_arm_s']}"]
    _show('\n'.join(md), '\n'.join(text))


def print_summary(run: Run) -> None:
    for key, value in summary(run).items():
        print(f'{key:>26}: {value}')


# ---------------------------------------------------------------- plotting

def _plt():
    import matplotlib.pyplot as plt
    return plt


def _mark_events(ax, run: Run):
    t_arm = run.arm_time_s
    if t_arm is not None:
        ax.axvline(t_arm, color='0.4', ls='--', lw=1)
    for p in run.pops():
        ax.axvline(p['t_s'], color=COLOURS.get(p['colour'], 'k'), lw=1.5, alpha=0.8)


def _balloon_diameter(run: Run) -> float:
    return float(run.extra.get('layout', {}).get('balloon', {}).get('diameter_m',
                                                                    BALLOON_DIAMETER_M))


def _pop_distance(run: Run) -> float:
    """Largest vehicle-to-balloon centre distance at which a pop can happen:
    balloon radius + the farthest reach of the vehicle's contact spheres."""
    spheres = run.extra.get('layout', {}).get('vehicle', {}).get('contact_spheres', [])
    reach = max((float(np.linalg.norm(c['position_flu_m'])) + c['radius_m'] for c in spheres),
                default=0.08)
    return _balloon_diameter(run) / 2 + reach


def plot_map(run: Run, ax=None):
    """Top view: flight path (ground truth), balloons, pops."""
    plt = _plt()
    ax = ax or plt.gca()
    tr = run.trajectory
    for b in run.balloons:
        x, y, _ = b['position']
        ax.add_patch(plt.Circle((x, y), _balloon_diameter(run) / 2, color=COLOURS[b['colour']],
                                alpha=0.35 if b['popped'] else 0.9, ec='k' if b['points'] < 0 else None,
                                lw=1.5 if b['points'] < 0 else 0))
        ax.annotate(f"{b['points']:+d}{' (popped)' if b['popped'] else ''}", (x, y),
                    textcoords='offset points', xytext=(8, 6), fontsize=8)
    if tr:
        ax.plot(tr['x_enu_m'], tr['y_enu_m'], color='k', lw=1.2, label='flight path')
        ax.plot(tr['x_enu_m'][0], tr['y_enu_m'][0], 'k^', ms=8, label='take-off')
        for p in run.pops():
            ax.plot(*p['position'][:2], 'x', color=COLOURS.get(p['colour'], 'k'), ms=12, mew=3)
    ax.set_aspect('equal')
    ax.set_xlabel('x east [m]')
    ax.set_ylabel('y north [m]')
    ax.set_title('top view (ground truth); x = pop')
    ax.grid(alpha=0.3)
    ax.legend(loc='lower left', fontsize=8)
    return ax


def plot_altitude(run: Run, ax=None):
    plt = _plt()
    ax = ax or plt.gca()
    tr, te = run.trajectory, run.telemetry
    if tr:
        ax.plot(tr['t_s'], tr['z_enu_m'], label='true height')
    if te:
        ax.plot(te['t_s'], te['altitude_m'], label='baro altitude (telemetry)', alpha=0.8)
    for b in run.balloons:
        if b['points'] > 0:
            ax.axhline(b['position'][2], color=COLOURS[b['colour']], lw=0.6, alpha=0.5)
    _mark_events(ax, run)
    ax.set_ylabel('height [m]')
    ax.set_title('altitude (dashed = armed, coloured = pops; thin lines = balloon heights)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    return ax


def plot_attitude(run: Run, axes=None):
    plt = _plt()
    if axes is None:
        _, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 6))
    tr, te = run.trajectory, run.telemetry
    if tr:
        axes[0].plot(tr['t_s'], np.degrees(tr['roll_rad']), label='true')
        # REP-103 pitch + = nose down; telemetry pitch + = nose up
        axes[1].plot(tr['t_s'], -np.degrees(tr['pitch_rad']), label='true')
        heading = (90.0 - np.degrees(tr['yaw_rad'])) % 360.0
        axes[2].plot(tr['t_s'], heading, '.', ms=1.5, label='true')
    if te:
        axes[0].plot(te['t_s'], te['roll_deg'], label='flight controller', alpha=0.8)
        axes[1].plot(te['t_s'], te['pitch_deg'], label='flight controller', alpha=0.8)
        axes[2].plot(te['t_s'], te['heading_deg'], '.', ms=1.5, label='flight controller', alpha=0.8)
    for ax, label in zip(axes, ['roll [deg]\n+ right down', 'pitch [deg]\n+ nose up',
                                'heading [deg]\nCW from north']):
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        _mark_events(ax, run)
    axes[0].legend(fontsize=8)
    axes[0].set_title('attitude: truth vs what the flight controller reported')
    axes[-1].set_xlabel('simulation time [s]')
    return axes


def plot_commands(run: Run, axes=None):
    plt = _plt()
    if axes is None:
        _, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 5))
    c = run.commands
    if c:
        for name in ('roll', 'pitch', 'yaw_rate'):
            axes[0].plot(c['t_s'], c[name], label=name)
        axes[1].plot(c['t_s'], c['throttle'], label='throttle', color='tab:red')
        axes[1].plot(c['t_s'], c['arm_switch'], label='arm switch', color='0.5', lw=0.8)
    axes[0].set_ylabel('stick [-1..1]')
    axes[0].set_title('commands that reached the drone (your controller + failsafe)')
    axes[1].set_ylabel('throttle [0..1]')
    axes[1].set_xlabel('simulation time [s]')
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        _mark_events(ax, run)
    return axes


def plot_balloon_distances(run: Run, ax=None):
    """Distance from the drone to every balloon over time."""
    plt = _plt()
    ax = ax or plt.gca()
    tr = run.trajectory
    if not tr:
        return ax
    p = np.stack([tr['x_enu_m'], tr['y_enu_m'], tr['z_enu_m']], axis=1)
    for b in run.balloons:
        d = np.linalg.norm(p - b['position'], axis=1)
        ax.plot(tr['t_s'], d, color=COLOURS[b['colour']], ls='-' if b['points'] > 0 else ':',
                label=f"{b['name']} ({b['points']:+d})")
    reach = _pop_distance(run)
    ax.axhline(reach, color='k', lw=0.8, ls='--')
    ax.text(ax.get_xlim()[0], reach + 0.02, ' max pop distance', fontsize=8)
    _mark_events(ax, run)
    ax.set_ylabel('distance [m]')
    ax.set_xlabel('simulation time [s]')
    ax.set_title('distance to each balloon (dotted = red, a penalty)')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)
    return ax


def plot_overview(run: Run):
    """The main figures of a run."""
    plt = _plt()
    fig = plt.figure(figsize=(14, 6))
    plot_map(run, fig.add_subplot(1, 2, 1))
    plot_altitude(run, fig.add_subplot(2, 2, 2))
    plot_balloon_distances(run, fig.add_subplot(2, 2, 4))
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------ camera

def frame_at(run: Run, t_s: float) -> Optional[np.ndarray]:
    """The recorded camera frame closest to simulation time t_s (RGB array)."""
    video = os.path.join(run.path, 'camera.mp4')
    cf = run.camera_frames
    if not os.path.isfile(video) or not cf or not len(cf['t_s']):
        return None
    if not np.isfinite(t_s):
        return None
    index = int(cf['frame'][np.argmin(np.abs(cf['t_s'] - t_s))])
    try:
        probe = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                                '-show_entries', 'stream=width,height', '-of', 'csv=p=0', video],
                               capture_output=True, text=True)
        width, height = (int(v) for v in probe.stdout.strip().split(','))
        raw = subprocess.run(['ffmpeg', '-v', 'error', '-i', video, '-vf',
                              f'select=eq(n\\,{index})', '-vsync', '0', '-frames:v', '1',
                              '-f', 'rawvideo', '-pix_fmt', 'rgb24', 'pipe:1'],
                             capture_output=True).stdout
    except (OSError, ValueError):   # no ffmpeg/ffprobe, or an unreadable video
        return None
    if len(raw) != width * height * 3:
        return None
    return np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)


def key_times(run: Run) -> List[tuple]:
    """(label, time) moments worth looking at: arm, before each pop, end."""
    times = []
    t_arm = run.arm_time_s
    if t_arm is not None:
        times.append(('armed', t_arm + 0.1))
    for p in run.pops():
        times.append((f"0.5 s before {p['colour']} pop", p['t_s'] - 0.5))
    end = run.end_time_s
    if end is not None:
        times.append(('end of run', end - 0.1))
    return times


def show_frames(run: Run, times: Optional[List[tuple]] = None, columns: int = 3):
    """Camera frames at key moments (default: key_times)."""
    plt = _plt()
    times = times or key_times(run)
    if not times:
        print('no camera frames to show')
        return None
    rows = (len(times) + columns - 1) // columns
    fig, axes = plt.subplots(rows, columns, figsize=(5 * columns, 3 * rows), squeeze=False)
    for ax in axes.flat:
        ax.axis('off')
    for ax, (label, t) in zip(axes.flat, times):
        image = frame_at(run, t)
        if image is not None:
            ax.imshow(image)
        ax.set_title(f'{label} (t = {t:.1f} s)', fontsize=9)
    fig.tight_layout()
    return fig
