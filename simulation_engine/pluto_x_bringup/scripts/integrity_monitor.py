#!/usr/bin/env python3
"""integrity_monitor: the fair-play check of a running controller.

Started by competition.launch.py for every run. A controller may use only
the avatic_drone API (camera frames, flight-controller telemetry, the
arena's score and events). This node watches for anything that reaches
the simulator's ground truth while the run is on:

  ROS graph (every second)
    - the controller's node (avatic_drone) subscribes to, publishes or
      calls anything outside the API;
    - another ROS program subscribes to anything outside the API (for
      example /sim/pluto/odometry or /arena/viz) or publishes /pluto/rc;
    - two nodes share an organiser node's name (a disguised node).
  controller process (every 0.2 s; only when the launch started the
  controller, controller:=...; found by a per-run token in its environment,
  inherited by anything it starts)
    - it started another program;
    - it loaded the Gazebo transport or SDFormat libraries (the simulator's
      own topics and services, e.g. every model's pose);
    - it has one of the run's files open (the layout, the world, the
      recording, an evaluation session) or another process's /proc files.

The findings are published on /arena/integrity (latched YAML: verdict
'clean' | 'flagged', checks, flags, notes); the recorder saves them in the
run's meta.yaml, and the analysis and evaluation notebooks show them. A
flag is for an organiser to review, not an automatic disqualification.
The code itself is checked by evaluation/check_controller.py.
"""

import os
import re
import time

import yaml

CONTROLLER_NODE = 'avatic_drone'
# what the avatic_drone API uses (outerloop_controller/avatic_drone/sim.py)
API_SUBSCRIPTIONS = {'/pluto/camera/image_raw', '/pluto/fc_status', '/arena/score',
                     '/arena/time_remaining', '/arena/events', '/arena/result', '/clock',
                     '/parameter_events'}
API_PUBLICATIONS = {'/pluto/rc', '/rosout', '/parameter_events'}
# topics no program other than the organiser's may publish
PROTECTED_PUBLICATIONS = ('/pluto/rc', '/arena/', '/sim/', '/clock')
GAZEBO_LIBRARY = re.compile(r'lib(?:gz-|ignition-|sdformat)[\w.+-]*\.so')
FORBIDDEN_FILES = ('pluto_arena', 'pluto_run_', '/analysis/runs/', '/evaluation/sessions/',
                   '.sdf', 'layout.yaml', 'arena.yaml', 'arena_default.yaml', 'seed.yaml')


def check_graph(nodes, subscriptions, publications, clients, organiser_nodes):
    """Flags and notes from the ROS graph.

    nodes: list of full node names (duplicates kept); subscriptions,
    publications, clients: {full node name: set of topic/service names};
    organiser_nodes: full names of the simulator's own nodes (a trailing
    '*' matches any ending)."""
    def organiser(name):
        return any(name == n or (n.endswith('*') and name.startswith(n[:-1]))
                   for n in organiser_nodes)
    flags, notes = [], []
    seen = set()
    for name in nodes:
        if name in seen and (organiser(name) or name == '/' + CONTROLLER_NODE):
            flags.append(f'two ROS nodes are named {name}: one of them is disguised')
        seen.add(name)
    for name in sorted(seen):
        if organiser(name):
            continue
        subs = subscriptions.get(name, set())
        pubs = publications.get(name, set())
        if name == '/' + CONTROLLER_NODE:
            extra = sorted(subs - API_SUBSCRIPTIONS)
            if extra:
                flags.append(f'the controller subscribes to {", ".join(extra)} '
                             '(not part of the avatic_drone API)')
            extra = sorted(pubs - API_PUBLICATIONS)
            if extra:
                flags.append(f'the controller publishes {", ".join(extra)} '
                             '(not part of the avatic_drone API)')
            calls = sorted(clients.get(name, set()))
            if calls:
                flags.append(f'the controller calls the service(s) {", ".join(calls)}')
            continue
        extra = sorted(subs - API_SUBSCRIPTIONS - {'/rosout'})
        if extra:
            flags.append(f'another ROS program ({name}) subscribes to {", ".join(extra)}')
        extra = sorted(t for t in pubs if t.startswith(PROTECTED_PUBLICATIONS))
        if extra:
            flags.append(f'another ROS program ({name}) publishes {", ".join(extra)}')
        if not name.startswith('/_ros2cli_daemon'):   # the ros2 command-line tool's daemon
            notes.append(f'another ROS program was running: {name} (judged runs have none)')
    return flags, notes


def controller_processes(token, proc='/proc'):
    """PIDs of the processes whose environment holds AVATIC_RUN_TOKEN=token."""
    marker = f'AVATIC_RUN_TOKEN={token}'.encode()
    pids = []
    for entry in os.listdir(proc):
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join(proc, entry, 'environ'), 'rb') as stream:
                if marker in stream.read().split(b'\0'):
                    pids.append(int(entry))
        except OSError:
            continue   # gone, or another user's
    return sorted(pids)


def parent_pid(pid, proc='/proc'):
    try:
        with open(os.path.join(proc, str(pid), 'stat'), encoding='utf-8') as stream:
            return int(stream.read().rsplit(')', 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        return None


def started_programs(pids, proc='/proc'):
    """The processes among pids started by another one of them (the
    controller itself is the one whose parent is not in the list)."""
    return [p for p in pids if parent_pid(p, proc) in pids] if len(pids) > 1 else []


def command_line(pid, proc='/proc'):
    try:
        with open(os.path.join(proc, str(pid), 'cmdline'), 'rb') as stream:
            return stream.read().replace(b'\0', b' ').decode(errors='replace').strip()
    except OSError:
        return '?'


def check_libraries(pid, proc='/proc'):
    """Flags for Gazebo / SDFormat libraries loaded by the process."""
    try:
        with open(os.path.join(proc, str(pid), 'maps'), encoding='utf-8',
                  errors='replace') as stream:
            found = sorted(set(GAZEBO_LIBRARY.findall(stream.read())))
    except OSError:
        return []
    if not found:
        return []
    return [f'the controller loaded the Gazebo libraries {", ".join(found[:4])} '
            '(the simulator\'s own topics and services)']


def check_open_files(pid, extra_paths=(), proc='/proc'):
    """Flags for run files (or other processes' /proc files) the process has open."""
    flags = []
    fd_dir = os.path.join(proc, str(pid), 'fd')
    try:
        fds = os.listdir(fd_dir)
    except OSError:
        return flags
    for fd in fds:
        try:
            target = os.readlink(os.path.join(fd_dir, fd))
        except OSError:
            continue
        if not target.startswith('/'):
            continue   # sockets, pipes
        # another process's files (/proc/<pid>/...), not /proc/cpuinfo and the like
        match = re.match(r'/proc/(\d+)/', target)
        other_proc = bool(match) and int(match.group(1)) != pid
        if other_proc or any(m in target for m in FORBIDDEN_FILES) or \
                any(p and target.startswith(p) for p in extra_paths):
            flags.append(f'the controller opened {target}')
    return flags


class Findings:
    """Flags and notes, each kept once, with the time it was first seen."""

    def __init__(self, checks):
        self.checks, self.flags, self.notes = list(checks), [], []
        self._seen = set()

    def add(self, t_s, flags=(), notes=()):
        new = []
        for kind, items in (('flag', flags), ('note', notes)):
            for text in items:
                if (kind, text) in self._seen:
                    continue
                self._seen.add((kind, text))
                entry = {'t_s': round(t_s, 2), 'text': text}
                (self.flags if kind == 'flag' else self.notes).append(entry)
                new.append((kind, text))
        return new

    def report(self):
        return {'verdict': 'flagged' if self.flags else 'clean', 'checks': self.checks,
                'flags': self.flags, 'notes': self.notes}


def main():
    import rclpy
    import rclpy.executors
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class IntegrityMonitor(Node):

        def __init__(self):
            super().__init__('integrity_monitor')
            self.token = self.declare_parameter('controller_token', '').value
            self.organiser = {n if n.startswith('/') else '/' + n for n in
                              self.declare_parameter('organiser_nodes', ['']).value if n}
            self.organiser.add('/integrity_monitor')
            self.watch_paths = [p for p in self.declare_parameter(
                'watch_paths', ['']).value if p]
            checks = ['ros graph'] + (['controller process'] if self.token else [])
            self.findings = Findings(checks)
            if not self.token:
                self.findings.add(0.0, notes=['the controller was not started by the launch '
                                              '(controller:=...): only the ROS checks ran'])
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, '/arena/integrity', qos)
            self.pids = []
            self.start = time.monotonic()
            wall = rclpy.clock.Clock()
            self.create_timer(1.0, self.check_slow, clock=wall)
            self.create_timer(0.2, self.check_fast, clock=wall)
            self.create_subscription(String, '/arena/result', self.on_result, 10)
            self.publish()

        def on_result(self, _msg):
            report = self.findings.report()
            text = f"[fair play] this run: {report['verdict']}"
            if report['flags']:
                text += ' - ' + '; '.join(f['text'] for f in report['flags'])
            (self.get_logger().warning if report['flags'] else self.get_logger().info)(text)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def record(self, flags=(), notes=()):
            new = self.findings.add(self.now(), flags, notes)
            for kind, text in new:
                if kind == 'flag':
                    self.get_logger().warning(f'[fair play] FLAG: {text}')
                else:
                    self.get_logger().info(f'[fair play] note: {text}')
            if new:
                self.publish()

        def publish(self):
            self.pub.publish(String(data=yaml.safe_dump(self.findings.report(),
                                                        sort_keys=False)))

        def check_slow(self):
            nodes, subs, pubs, clients = [], {}, {}, {}
            for name, namespace in self.get_node_names_and_namespaces():
                full = (namespace.rstrip('/') + '/' + name) if namespace != '/' else '/' + name
                nodes.append(full)
                try:
                    subs[full] = {t for t, _ in
                                  self.get_subscriber_names_and_types_by_node(name, namespace)}
                    pubs[full] = {t for t, _ in
                                  self.get_publisher_names_and_types_by_node(name, namespace)}
                    clients[full] = {s for s, _ in
                                     self.get_client_names_and_types_by_node(name, namespace)}
                except Exception:   # noqa: BLE001 - the node left meanwhile
                    continue
            self.record(*check_graph(nodes, subs, pubs, clients, self.organiser))
            if self.token:
                self.pids = controller_processes(self.token)
                self.record([f'the controller started another program: {command_line(p)}'
                             for p in started_programs(self.pids)])
                for pid in self.pids:
                    self.record(check_libraries(pid))

        def check_fast(self):
            for pid in self.pids:
                self.record(check_open_files(pid, self.watch_paths))

    rclpy.init()
    node = IntegrityMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
