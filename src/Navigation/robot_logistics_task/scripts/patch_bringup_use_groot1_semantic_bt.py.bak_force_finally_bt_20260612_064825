#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pathlib import Path
import yaml

p = Path('/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml')
module = {
    'type': 'node',
    'package': 'robot_logistics_task',
    'executable': 'mission_btcpp_node',
    'name': 'mission_btcpp_node',
    'output': 'screen',
    'delay': 11.0,
    'parameters': [{
        'bt_xml': '/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_full_groot1.xml',
        'semantic_map_path': '/host-home/maps/semantic_library.json',
        'semantic_session_path': '/host-home/maps/semantic_session_pose.json',
        'map_session_id': 'current_slam_session',
        'shelf_goal_topic': '/task/shelf_goal',
        'cancel_topic': '/task/cancel',
        'task_status_topic': '/task/status',
        'current_state_topic': '/task/current_state',
        'current_goal_topic': '/task/current_goal',
        'bt_trace_topic': '/task/bt_trace',
        'bt_xml_topic': '/task/bt_tree_xml',
        'goal_pose_topic': '/goal_pose',
        'platform_target_topic': '/platform/target',
        'platform_status_topic': '/platform/status',
        'explore_cmd_topic': '/task/explore_cmd',
        'control_mode_topic': '/control_mode',
        'anchor_detection_topic': '/semantic/anchor_detection',
        'current_shelf_topic': '/semantic/current_shelf',
        'map_frame': 'map',
        'base_frame': 'base_link',
        'session_pose_max_age_sec': 600.0,
        'min_session_confidence': 0.50,
        'allow_demo_anchor_if_missing': True,
        'enable_groot_monitoring': True,
        'groot_publisher_port': 1666,
        'groot_server_port': 1667,
        'groot_max_msg_per_second': 25,
    }],
}
if not p.exists():
    raise SystemExit(f'bringup yaml not found: {p}')
cfg = yaml.safe_load(p.read_text(encoding='utf-8'))
cfg.setdefault('modules', {})['mission_bt'] = module
cfg['modules'].pop('mission_btcpp', None)
task = cfg.setdefault('modes', {}).setdefault('task', {}).setdefault('enable', [])
task = [x for x in task if x not in ('mission_btcpp', 'mission_behavior_tree_node')]
if 'mission_bt' not in task:
    task.insert(task.index('semantic_registrar') + 1, 'mission_bt') if 'semantic_registrar' in task else task.append('mission_bt')
cfg['modes']['task']['enable'] = task
p.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding='utf-8')
print(f'patched: {p}')
print('task enable:', cfg['modes']['task']['enable'])
print('Groot v1 ports: publisher=1666 server=1667')
