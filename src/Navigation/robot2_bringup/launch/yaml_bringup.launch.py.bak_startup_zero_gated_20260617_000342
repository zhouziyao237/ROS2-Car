#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YAML-driven bringup for robot2.

Main usage:
  ros2 launch robot2_bringup yaml_bringup.launch.py mode:=mapping
  ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task
  ros2 launch robot2_bringup yaml_bringup.launch.py mode:=hardware disable_modules:=lidar,chassis_serial

YAML supports:
  type: node
  type: include

Special parameter string support:
  "$(file /abs/path/file.urdf)"   -> read file content as string
  "$(cat /abs/path/file.urdf)"    -> read file content as string
  "$(xacro /abs/path/file.xacro)" -> execute xacro command substitution
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node


def _as_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _split_csv(value):
    if value is None:
        return []
    return [x.strip() for x in str(value).split(",") if x.strip()]


def _read_text_file(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = f.read()

    if not data.strip():
        raise RuntimeError(f"file is empty: {path}")

    return data


def _convert_param_value(value):
    """
    Convert special YAML string syntax into actual launch parameter values.

    Most important case:
      robot_description: "$(file /path/to/robot.urdf)"

    Without this conversion, robot_state_publisher receives an invalid or empty
    robot_description and crashes with:
      Error document empty.
    """

    if isinstance(value, str):
        text = value.strip()

        if text.startswith("$(file ") and text.endswith(")"):
            path = text[len("$(file "):-1].strip()
            return _read_text_file(path)

        if text.startswith("$(cat ") and text.endswith(")"):
            path = text[len("$(cat "):-1].strip()
            return _read_text_file(path)

        if text.startswith("$(xacro ") and text.endswith(")"):
            path = text[len("$(xacro "):-1].strip()
            if not os.path.exists(path):
                raise FileNotFoundError(f"xacro file not found: {path}")
            return Command(["xacro ", path])

        return value

    if isinstance(value, dict):
        return {k: _convert_param_value(v) for k, v in value.items()}

    if isinstance(value, list):
        return [_convert_param_value(v) for v in value]

    return value


def _normalise_parameters(params):
    """
    YAML accepted formats:

    parameters:
      - /path/to/params.yaml
      - key: value

    or:

    parameters:
      key: value
    """

    if params is None:
        return []

    if isinstance(params, list):
        return [_convert_param_value(x) for x in params]

    if isinstance(params, dict):
        return [_convert_param_value(params)]

    return [_convert_param_value(params)]


def _resolve_launch_path(package_name, launch_name):
    """
    Resolve include launch path.

    YAML:
      package: nav2_bringup
      launch: navigation_launch.py

    becomes:
      <share/nav2_bringup>/launch/navigation_launch.py
    """

    if os.path.isabs(launch_name):
        return launch_name

    return os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_name,
    )


def _module_to_action(module_name, module_cfg):
    if module_cfg is None:
        raise RuntimeError(
            f"Module '{module_name}' is empty in bringup_modes.yaml. "
            f"Check YAML indentation."
        )

    if not isinstance(module_cfg, dict):
        raise RuntimeError(
            f"Module '{module_name}' must be a YAML dictionary, got: {type(module_cfg)}"
        )

    mtype = module_cfg.get("type", "node")
    delay = float(module_cfg.get("delay", 0.0))

    if mtype == "node":
        if "package" not in module_cfg:
            raise RuntimeError(f"Module '{module_name}' missing 'package'")
        if "executable" not in module_cfg:
            raise RuntimeError(f"Module '{module_name}' missing 'executable'")

        action = Node(
            package=module_cfg["package"],
            executable=module_cfg["executable"],
            name=module_cfg.get("name", module_name),
            namespace=module_cfg.get("namespace", ""),
            output=module_cfg.get("output", "screen"),
            parameters=_normalise_parameters(module_cfg.get("parameters")),
            remappings=[tuple(x) for x in module_cfg.get("remappings", [])],
            arguments=[str(x) for x in module_cfg.get("arguments_list", [])],
            respawn=_as_bool(module_cfg.get("respawn", False)),
            respawn_delay=float(module_cfg.get("respawn_delay", 2.0)),
        )

    elif mtype == "include":
        if "package" not in module_cfg:
            raise RuntimeError(f"Include module '{module_name}' missing 'package'")
        if "launch" not in module_cfg:
            raise RuntimeError(f"Include module '{module_name}' missing 'launch'")

        launch_path = _resolve_launch_path(
            module_cfg["package"],
            module_cfg["launch"],
        )

        if not os.path.exists(launch_path):
            raise FileNotFoundError(
                f"Launch file for module '{module_name}' not found: {launch_path}"
            )

        action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                str(k): str(v)
                for k, v in module_cfg.get("arguments", {}).items()
            }.items(),
        )

    else:
        raise RuntimeError(
            f"Unknown module type '{mtype}' for module '{module_name}'. "
            f"Supported types: node, include"
        )

    if delay > 0.0:
        return TimerAction(
            period=delay,
            actions=[
                LogInfo(msg=f"[robot2_bringup] start module after {delay:.1f}s: {module_name}"),
                action,
            ],
        )

    return action


def _runtime_setup(context, *args, **kwargs):
    mode_value = LaunchConfiguration("mode").perform(context)
    config_value = LaunchConfiguration("config_file").perform(context)
    enable_extra = _split_csv(LaunchConfiguration("enable_modules").perform(context))
    disable_list = set(_split_csv(LaunchConfiguration("disable_modules").perform(context)))

    if not os.path.exists(config_value):
        raise FileNotFoundError(f"bringup config file not found: {config_value}")

    with open(config_value, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if cfg is None:
        raise RuntimeError(f"bringup config is empty: {config_value}")

    modules = cfg.get("modules", {})
    modes = cfg.get("modes", {})

    if mode_value not in modes:
        valid = ", ".join(sorted(modes.keys()))
        raise RuntimeError(f"Unknown mode '{mode_value}'. Valid modes: {valid}")

    enabled = list(modes[mode_value].get("enable", []))

    for module_name in enable_extra:
        if module_name not in enabled:
            enabled.append(module_name)

    enabled = [m for m in enabled if m not in disable_list]

    empty_modules = [k for k, v in modules.items() if v is None]
    if empty_modules:
        raise RuntimeError(
            f"Empty modules in bringup_modes.yaml: {empty_modules}. "
            f"This is usually caused by bad YAML indentation."
        )

    actions = [
        LogInfo(msg=f"[robot2_bringup] mode={mode_value}"),
        LogInfo(msg=f"[robot2_bringup] config={config_value}"),
        LogInfo(msg=f"[robot2_bringup] enabled modules={enabled}"),
    ]

    for module_name in enabled:
        if module_name not in modules:
            raise RuntimeError(
                f"Mode '{mode_value}' enables unknown module '{module_name}'"
            )

        actions.append(_module_to_action(module_name, modules[module_name]))

    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("robot2_bringup"),
        "config",
        "bringup_modes.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="hardware",
            description="Bringup mode name from bringup_modes.yaml",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to YAML bringup configuration",
        ),
        DeclareLaunchArgument(
            "enable_modules",
            default_value="",
            description="Comma-separated extra modules to enable",
        ),
        DeclareLaunchArgument(
            "disable_modules",
            default_value="",
            description="Comma-separated modules to disable",
        ),
        OpaqueFunction(function=_runtime_setup),
    ])
