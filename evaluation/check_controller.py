#!/usr/bin/env python3
"""Fair-play check of a controller's source code (no simulator needed).

    python3 evaluation/check_controller.py outerloop_controller/my_controller.py

A controller may use only what the avatic_drone API gives it: the camera
frames and the flight controller's telemetry. This scans the controller
file, and the .py files of its folder that it imports, for anything that
could reach the simulator's ground truth: other ROS or Gazebo interfaces,
the run's files, other processes, the network, or code that hides what it
does. It reads the code only; it never runs it.

Each finding is a line to be READ BY A PERSON, not a verdict: a finding in
harmless code (reading your own parameter file, say) is fine once
explained. Exit code 0 = nothing found, 1 = findings, 2 = a file could
not be read. evaluate.py runs this at the start of every session; the
simulator also watches the running controller (integrity_monitor.py).
"""

import argparse
import ast
import os
import sys
from typing import List, NamedTuple, Optional

# imports a controller may use freely (top-level package names)
ALLOWED_IMPORTS = {
    'avatic_drone', 'numpy', 'cv2', 'scipy', 'math', 'cmath', 'time', 'collections',
    'dataclasses', 'typing', 'enum', 'functools', 'itertools', 'operator', 'statistics',
    'random', 'heapq', 'bisect', 'copy', 'abc', 'argparse', 'sys', 'os', 'traceback',
    'logging', 'warnings', 'contextlib', '__future__', 'fractions', 'decimal', 'numbers',
    'string', 're', 'textwrap', 'pprint', 'json', 'queue', 'threading', 'types',
    'datetime', 'csv', 'yaml',
    # learning-based and vision libraries (encouraged; model files are read with a
    # 'review' note, fine for files in your own folder)
    'torch', 'torchvision', 'torch_geometric', 'onnx', 'onnxruntime', 'sklearn',
    'ultralytics', 'tensorflow', 'keras', 'skimage', 'PIL', 'numba', 'filterpy',
    'networkx', 'shapely', 'joblib',
}
# imports that reach the simulator, other processes or the network
FORBIDDEN_IMPORTS = {
    'rclpy', 'rclcpp', 'rosidl_runtime_py', 'rosbag2_py', 'ros2cli', 'launch', 'launch_ros',
    'ament_index_python', 'gz', 'ignition', 'sdformat', 'sdformat13', 'subprocess',
    'socket', 'socketserver', 'ssl', 'http', 'urllib', 'requests', 'ctypes', 'cffi',
    'multiprocessing', 'concurrent', 'pty', 'importlib', 'imp', 'runpy', 'zmq',
    'cyclonedds', 'fastdds', 'signal', 'mmap', 'shutil', 'glob', 'tempfile',
    'marshal', 'shelve', 'base64', 'codecs', 'zlib', 'psutil', 'inspect',
    'builtins', 'gc',
}
FORBIDDEN_IMPORT_SUFFIXES = ('_msgs', '_srvs', '_interfaces')    # ROS message packages
# calls that run code from strings, or hide which name is used
FORBIDDEN_CALLS = {'exec', 'eval', 'compile', '__import__', 'globals', 'locals', 'vars',
                   'getattr', 'setattr', 'delattr', 'breakpoint', 'os.system', 'os.popen'}
# calls that read files (a controller normally reads none)
FILE_CALLS = {'open'}
FILE_METHODS = {'load', 'loadtxt', 'genfromtxt', 'fromfile', 'imread', 'VideoCapture',
                'read_text', 'read_bytes', 'safe_load', 'load_all'}
# os functions that list or read files, touch other processes or the environment
OS_ALLOWED = {'path', 'environ', 'getcwd', 'sep', 'linesep', 'makedirs', 'mkdir', 'cpu_count'}
OS_PATH_ALLOWED = {'join', 'dirname', 'basename', 'abspath', 'splitext', 'normpath'}
# text that names the simulator's ground truth or the run's files
SUSPICIOUS_TEXT = (
    '/sim/', 'odometry', 'layout', '.sdf', 'seed', '/proc', 'analysis/runs', 'runs/',
    'evaluation/sessions', 'sessions/', 'pluto_arena', 'pluto_run', 'gz_partition',
    'ros_domain', 'pose/info', 'scene/info', 'dynamic_pose', '/world/', 'balloon_arena',
    '/arena/viz', '/pluto/imu', '/pluto/battery', 'trajectory.csv', 'meta.yaml',
    'result.yaml', 'session.yaml', 'arena_default', '/tmp', 'model/pluto_x', 'tf_static',
)
AVATIC_API = {'Drone', 'Command', 'Frame', 'Telemetry', 'ArenaStatus', 'connect',
              'SAFETY_LIMITS', 'apply_safety_limits'}   # avatic_drone's public names
OFFICIAL_AVATIC_DRONE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                     'outerloop_controller', 'avatic_drone')
BUILTIN_NAMES = {'__builtins__', '__dict__', '__class__', '__subclasses__', '__globals__',
                 '__code__', '__loader__', '__spec__', 'sys.modules', '__bases__', '__mro__'}


class Finding(NamedTuple):
    path: str
    line: int
    kind: str       # 'forbidden' (against the rules as written) or 'review'
    text: str

    def __str__(self):
        return f'{self.path}:{self.line}  [{self.kind}] {self.text}'


def _dotted(node) -> Optional[str]:
    """'a.b.c' for a Name/Attribute chain, else None."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
        return '.'.join(reversed(parts))
    return None


class _Scanner(ast.NodeVisitor):

    def __init__(self, path: str, folder: str):
        self.path, self.folder = path, folder
        self.findings: List[Finding] = []
        self.local_imports: List[str] = []   # .py files of the controller's folder
        self.os_names = {'os'}               # names bound to the os module

    def add(self, node, kind, text):
        self.findings.append(Finding(self.path, getattr(node, 'lineno', 0), kind, text))

    def _local(self, module: str) -> Optional[str]:
        candidate = os.path.join(self.folder, *module.split('.'))
        for path in (candidate + '.py', os.path.join(candidate, '__init__.py')):
            if os.path.isfile(path):
                return path
        return None

    def _check_import(self, node, module: str):
        top = module.split('.')[0]
        local = self._local(module)
        if top == 'avatic_drone' and self._local('avatic_drone') and \
                os.path.realpath(os.path.join(self.folder, 'avatic_drone')) != \
                os.path.realpath(OFFICIAL_AVATIC_DRONE):
            self.add(node, 'forbidden', 'a copy of avatic_drone next to the controller: it is '
                                        'imported instead of the official one')
            return
        if top == 'avatic_drone' and module != 'avatic_drone' and \
                module not in ('avatic_drone.types',):
            self.add(node, 'review', f'import {module}: the backend\'s internals (use '
                                     '"from avatic_drone import ...")')
            return
        if local and top != 'avatic_drone':
            self.local_imports.append(local)
        elif top in FORBIDDEN_IMPORTS or top.endswith(FORBIDDEN_IMPORT_SUFFIXES):
            self.add(node, 'forbidden', f'import {module}: not allowed in a controller '
                                        '(reaches the simulator, files, processes or the network)')
        elif top not in ALLOWED_IMPORTS and not local:
            self.add(node, 'review', f'import {module}: not on the allowed list')

    def visit_Import(self, node):
        for alias in node.names:
            self._check_import(node, alias.name)
            if alias.name == 'os':
                self.os_names.add(alias.asname or 'os')
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        if node.level:          # from . import x: a file of this package
            base = os.path.join(self.folder, *(['..'] * (node.level - 1)))
            for alias in node.names:
                name = os.path.join(base, *(node.module or '').split('.'), alias.name)
                for path in (name + '.py', os.path.join(name, '__init__.py')):
                    if os.path.isfile(os.path.normpath(path)):
                        self.local_imports.append(os.path.normpath(path))
            if node.module:
                name = os.path.join(base, *node.module.split('.'))
                for path in (name + '.py', os.path.join(name, '__init__.py')):
                    if os.path.isfile(os.path.normpath(path)):
                        self.local_imports.append(os.path.normpath(path))
        else:
            self._check_import(node, node.module or '')
            if node.module == 'os':
                self.add(node, 'review', 'from os import ...: use os.<name> so it can be checked')
            for alias in node.names:      # the imported names themselves
                name = alias.name
                if name in FORBIDDEN_IMPORTS or name in FORBIDDEN_CALLS or \
                        name in BUILTIN_NAMES:
                    self.add(node, 'forbidden', f'from {node.module} import {name}')
                elif (node.module or '').split('.')[0] == 'avatic_drone' and \
                        name not in AVATIC_API and name != '*':
                    self.add(node, 'review', f'from {node.module} import {name}: not part of '
                                             'the avatic_drone API')
                elif name in FILE_METHODS or name in FILE_CALLS:
                    self.add(node, 'review', f'from {node.module} import {name}: reads files')
        self.generic_visit(node)

    def visit_Call(self, node):
        name = _dotted(node.func)
        short = name.split('.')[-1] if name else None
        if name in FORBIDDEN_CALLS:
            self.add(node, 'forbidden', f'{name}(): runs code from text or hides which name '
                                        'is used')
        elif name in FILE_CALLS or (short in FILE_METHODS and name != short):
            self.add(node, 'review', f'{name}(): reads a file (fine for your own files, e.g. '
                                     'model weights or parameters: say which in your report)')
        if name in ('os.environ.get', 'os.getenv') or (name or '').endswith('environ.get'):
            key = node.args[0] if node.args else None
            if not (isinstance(key, ast.Constant) and str(key.value).startswith('AVATIC_')):
                self.add(node, 'review', f'{name}(): reads an environment variable other '
                                         'than AVATIC_*')
        self.generic_visit(node)

    def visit_Attribute(self, node):
        name = _dotted(node) if not isinstance(getattr(node, 'parent', None),
                                               ast.Attribute) else None   # whole chains only
        if name:
            parts = name.split('.')
            if parts[0] in self.os_names and len(parts) > 1:
                if parts[1] not in OS_ALLOWED:
                    self.add(node, 'forbidden', f'{name}: lists or reads files, or touches '
                                                'other processes')
                elif parts[1] == 'path' and len(parts) > 2 and parts[2] not in OS_PATH_ALLOWED:
                    self.add(node, 'review', f'{name}: looks at files')
                elif parts[1] == 'environ' and name != f'{parts[0]}.environ.get':
                    self.add(node, 'review', f'{name}: reads the environment')
            if parts[0] == 'avatic_drone' and len(parts) > 1 and parts[1] not in AVATIC_API:
                self.add(node, 'review', f'{name}: the backend\'s internals, not the API')
            if parts[0] == 'sys' and len(parts) > 1 and parts[1] in ('modules', '_getframe'):
                self.add(node, 'review', f'{name}: changes or inspects the loaded code')
        if node.attr.startswith('_') and not node.attr.startswith('__') and not (
                isinstance(node.value, ast.Name) and node.value.id in ('self', 'cls')):
            self.add(node, 'review', f'.{node.attr}: private attribute of another object '
                                     '(for example the Drone\'s internals)')
        if node.attr in BUILTIN_NAMES:
            self.add(node, 'forbidden', f'.{node.attr}: reaches the interpreter\'s internals')
        self.generic_visit(node)

    def visit_Name(self, node):
        if node.id in BUILTIN_NAMES:
            self.add(node, 'forbidden', f'{node.id}: reaches the interpreter\'s internals')
        called = isinstance(getattr(node, 'parent', None), ast.Call) and node.parent.func is node
        if not called:   # a builtin passed around under another name (calls: visit_Call)
            if node.id in FORBIDDEN_CALLS:
                self.add(node, 'forbidden', f'{node.id} used as a value (called under another '
                                            'name?)')
            elif node.id in FILE_CALLS:
                self.add(node, 'review', f'{node.id} used as a value: reads files under '
                                         'another name?')
        self.generic_visit(node)

    def visit_Constant(self, node):
        if isinstance(node.value, (str, bytes)):
            text = node.value.decode('latin-1') if isinstance(node.value, bytes) else node.value
            low = text.lower()
            hits = [s for s in SUSPICIOUS_TEXT if s in low]
            if hits:
                shown = text if len(text) <= 60 else text[:57] + '...'
                self.add(node, 'review', f'text {shown!r} mentions {", ".join(hits)}')
        self.generic_visit(node)


def check_file(path: str) -> (List[Finding], List[str]):
    """Findings in one file, and the local .py files it imports."""
    with open(path, encoding='utf-8') as stream:
        source = stream.read()
    try:
        tree = ast.parse(source, filename=path)
    except SyntaxError as err:
        return [Finding(path, err.lineno or 0, 'review', f'does not parse: {err.msg}')], []
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            child.parent = parent
    scanner = _Scanner(path, os.path.dirname(os.path.abspath(path)))
    scanner.visit(tree)
    # docstrings and comments explain; only code counts (drop docstring hits)
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            body = getattr(node, 'body', [])
            if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
                    and isinstance(body[0].value.value, str):
                docstrings.update(range(body[0].lineno, body[0].end_lineno + 1))
    findings = [f for f in dict.fromkeys(scanner.findings)
                if not (f.text.startswith('text ') and f.line in docstrings)]
    return findings, scanner.local_imports


def check_controller(path: str) -> List[Finding]:
    """Findings in the controller and every local file it imports."""
    findings, todo, seen = [], [os.path.abspath(path)], set()
    while todo:
        current = todo.pop()
        if current in seen:
            continue
        seen.add(current)
        found, imports = check_file(current)
        findings += found
        todo += [os.path.abspath(p) for p in imports]
    root = os.path.dirname(os.path.abspath(path))
    return [f._replace(path=os.path.relpath(f.path, root) if f.path.startswith(root)
                       else f.path) for f in findings]


def verdict(findings: List[Finding]) -> str:
    return 'clean' if not findings else 'review'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('controller', nargs='+', help='controller .py file(s)')
    args = parser.parse_args()
    status = 0
    for path in args.controller:
        try:
            findings = check_controller(path)
        except OSError as err:
            print(f'{path}: cannot read ({err})')
            status = 2
            continue
        if not findings:
            print(f'{path}: clean (nothing found)')
            continue
        status = max(status, 1)
        forbidden = sum(f.kind == 'forbidden' for f in findings)
        print(f'{path}: {len(findings)} finding(s), {forbidden} forbidden - a person must '
              'read these lines:')
        for finding in findings:
            print(f'  {finding}')
    sys.exit(status)


if __name__ == '__main__':
    main()
