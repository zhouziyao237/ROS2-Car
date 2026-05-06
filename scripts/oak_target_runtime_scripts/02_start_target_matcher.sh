#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 run target_matcher target_matcher_node
