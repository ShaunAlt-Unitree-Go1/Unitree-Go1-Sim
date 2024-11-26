# =============================================================================
# Unitree Go1 Simulation Dockerfile
# Created by: Shaun Altmann
# 
# Used to create a docker image capable of independently running a simulation
# of the Unitree Go1 robot with LiDAR, Camera Point Clouds, SLAM, and the ROS
# Navigation Stack.
# =============================================================================

# =============================================================================
# ROS Noetic and Ubuntu 20.04 Base
# =============================================================================
FROM ros:noetic-ros-base-focal


# =============================================================================
# Add User (rosuser)
# =============================================================================
# Run `useradd --help` for more information on adding a user
RUN useradd \
    -rm \
    -d /home/ubuntu \
    -s /bin/bash \
    -g root \
    -G sudo \
    -u 1000 rosuser \
    -p rosuser
USER rosuser
WORKDIR /home/rosuser


# =============================================================================
# Update Debian Packages
# =============================================================================
RUN sudo apt update -y
RUN sudo apt upgrade -y


# =============================================================================
# Updating ROS Dependencies
# =============================================================================
RUN rosdep update --rosdistro noetic


# =============================================================================
# Clone GitHub Repo
# =============================================================================


# =============================================================================
# End of File
# =============================================================================
