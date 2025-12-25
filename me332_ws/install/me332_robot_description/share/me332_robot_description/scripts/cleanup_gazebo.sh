#!/usr/bin/env bash
set -e

pkill -u "$USER" -TERM -x gzserver || true
pkill -u "$USER" -TERM -x gzclient || true
pkill -u "$USER" -TERM -x gazebo   || true
pkill -u "$USER" -TERM -f "spawn_entity.py" || true

sleep 0.3

pkill -u "$USER" -KILL -x gzserver || true
pkill -u "$USER" -KILL -x gzclient || true
pkill -u "$USER" -KILL -x gazebo   || true
pkill -u "$USER" -KILL -f "spawn_entity.py" || true

exit 0

