#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
xhost +
docker run -it --rm \
  --name ov-bash \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ov-env:latest \
  bash