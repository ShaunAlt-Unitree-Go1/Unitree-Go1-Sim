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
# Prevent User Interaction
# =============================================================================
ENV DEBIAN_FRONTEND=noninteractive


# =============================================================================
# Update Debian Packages
# =============================================================================
RUN apt-get update -y
RUN apt-get upgrade -y


# =============================================================================
# Install Debian Packages
# =============================================================================
RUN apt-get install git -y
RUN apt-get install python3-catkin-tools -y


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
WORKDIR /home/rosuser


# =============================================================================
# Updating ROS Dependencies
# =============================================================================
RUN rosdep update --rosdistro noetic


# =============================================================================
# Clone + Build Simulation from GitHub Repo
# =============================================================================
RUN mkdir -p ~/sim_ws/src
WORKDIR /home/rosuser/sim_ws/src
RUN git clone \
    --recurse-submodules \
    https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
WORKDIR /home/rosuser/sim_ws
# RUN rosdep install \
#     --from-paths src \
#     --ignore-src \
#     --rosdistro noetic \
#     -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


# =============================================================================
# Set Default User
# =============================================================================
USER rosuser


# =============================================================================
# End of File
# =============================================================================
