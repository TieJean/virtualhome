#!/bin/bash
# set -x

# Tell OpenGL to use indirect rendering with real GPU
# export LIBGL_ALWAYS_INDIRECT=1
# export LIBGL_ALWAYS_SOFTWARE=1

# 640 480 Beautiful
# xvfb-run --auto-servernum --server-args="-screen 0 640x480x24" \
    # /unity_vol/linux_exec.v2.3.0.x86_64 -batchmode -http-port=8080 -screen-quality 4

Xvfb :99 -screen 0 640x480x24 &
export DISPLAY=:99

# /unity_vol/linux_exec.v2.3.0.x86_64 -batchmode -http-port=8080 -screen-fullscreen 0 -screen-quality 4 -logfile /dev/stdout

echo "Unity executable finished running."
