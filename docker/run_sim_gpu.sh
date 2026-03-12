#!/bin/bash
# podman stop virtualhome_container && podman rm virtualhome_container &&  podman run --name virtualhome_container   --hooks-dir=/usr/share/containers/oci/hooks.d   --env NVIDIA_VISIBLE_DEVICES=all   --env NVIDIA_DRIVER_CAPABILITIES=all,graphics,display,compute,utility   --device /dev/nvidia0   --device /dev/nvidiactl   --device /dev/nvidia-uvm   --mount type=bind,source="$(pwd)"/unity_vol,target=/unity_vol/   --mount type=bind,source="$(pwd)"/unity_output,target=/Output/   -p 28080:8080 --entrypoint /bin/bash  -it virtualhome



Xvfb :99 -screen 0 640x480x24 &
# export DISPLAY=:99

# export DISPLAY=:99 && /unity_vol/linux_exec.v2.3.0.x86_64 -batchmode -http-port=8080 -screen-fullscreen 0 -screen-quality 4 -logfile /dev/stdout 
export DISPLAY=:99 && (/unity_vol/linux_exec.v2.3.0.x86_64 -batchmode -http-port=8080 -screen-fullscreen 0 -screen-quality 4 -logfile /dev/stdout) &
sleep 5
kill -INT $!

export __GLX_VENDOR_LIBRARY_NAME=nvidia
export EGL_PLATFORM=device
# Point to the NVIDIA internal libraries first
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia:$LD_LIBRARY_PATH

/unity_vol/linux_exec.v2.3.0.x86_64 -batchmode -platform-egl -force-opengl -http-port=8080 -screen-fullscreen 0 -screen-quality 4 -logfile /dev/stdout