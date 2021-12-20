#!/bin/bash
# For default we share the network with the host as well as the display to allow GUI applications.
# The container is started interactively and removed when finished
# Add additional flags if required.
docker run \
-it --rm \
--network=host \
--cap-add=SYS_PTRACE \
--security-opt=seccomp:unconfined \
--security-opt=apparmor:unconfined \
--volume=/tmp/.X11-unix:/tmp/.X11-unix \
-v$(pwd):/workspace \
stereo_calibration "$@"
