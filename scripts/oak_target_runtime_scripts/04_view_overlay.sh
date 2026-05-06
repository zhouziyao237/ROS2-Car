#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 run rqt_image_view rqt_image_view /target_position_overlay/image
