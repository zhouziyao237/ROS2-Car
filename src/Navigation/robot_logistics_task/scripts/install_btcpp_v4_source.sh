#!/usr/bin/env bash
set -e
cd /host-home/src/Navigation
if [ ! -d behaviortree_cpp ]; then
  git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git behaviortree_cpp
else
  echo "behaviortree_cpp already exists, skip clone"
fi
