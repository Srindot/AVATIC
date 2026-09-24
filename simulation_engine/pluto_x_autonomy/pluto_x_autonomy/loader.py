"""Loads an outer-loop controller class from a specification string.

Accepted forms:
  package.module:ClassName     importable module (installed package, or on
                               PYTHONPATH)
  /path/to/file.py:ClassName   a plain Python file (no package needed)
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
from typing import Any, Mapping, Type

import yaml

from .api import OuterLoopController


def load_controller_class(spec: str) -> Type[OuterLoopController]:
    if ':' not in spec:
        raise ValueError(
            f"controller '{spec}' must be 'module:Class' or '/file.py:Class'")
    location, class_name = spec.rsplit(':', 1)
    if location.endswith('.py'):
        path = os.path.abspath(os.path.expanduser(location))
        if not os.path.isfile(path):
            raise ValueError(f'controller file not found: {path}')
        module_name = f'_pluto_outer_loop_{abs(hash(path))}'
        module_spec = importlib.util.spec_from_file_location(module_name, path)
        if module_spec is None or module_spec.loader is None:
            raise ValueError(f'cannot import {path}')
        module = importlib.util.module_from_spec(module_spec)
        sys.modules[module_name] = module
        module_spec.loader.exec_module(module)
    else:
        module = importlib.import_module(location)
    cls = getattr(module, class_name, None)
    if cls is None:
        raise ValueError(f"'{class_name}' not found in {location}")
    if not (isinstance(cls, type) and issubclass(cls, OuterLoopController)):
        raise ValueError(
            f'{spec} is not a subclass of pluto_x_autonomy.api.'
            'OuterLoopController')
    return cls


def load_params(path: str) -> Mapping[str, Any]:
    """Reads a YAML mapping of controller parameters ('' = none)."""
    if not path:
        return {}
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f'controller parameters in {path} must be a mapping')
    return data


def make_controller(spec: str, params_path: str) -> OuterLoopController:
    return load_controller_class(spec)(load_params(params_path))
