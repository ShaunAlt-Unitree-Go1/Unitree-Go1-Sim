#!/bin/bash
# =============================================================================
# Unitree Go1 Simulation Docker Image Run
# Created by: Shaun Altmann
#
# Used to run the pre-made docker image with the required settings.
# =============================================================================


# =============================================================================
# Run the Docker Image
# =============================================================================
docker run \
    -it \
    --net=host \
    --ipc=host \
    --device=/dev/dri:/dev/dri \
    --name=go1_sim_noetic \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    go1-sim-noetic


# =============================================================================
# End of File
# =============================================================================
