#!/bin/bash
# =============================================================================
# Unitree Go1 Simulation Docker Image Builder
# Created by: Shaun Altmann
#
# Used to build the docker image with the required settings.
# =============================================================================


# =============================================================================
# Build the Docker Image
# =============================================================================
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "Script directory: $SCRIPT_DIR"
echo "\$0 $0"
docker build -t go1-sim-noetic $0/.


# =============================================================================
# End of File
# =============================================================================
