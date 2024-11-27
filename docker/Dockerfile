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
# Force Software Rendering
# =============================================================================
# ENV LIBGL_ALWAYS_SOFTWARE=1


# =============================================================================
# Update Debian Packages
# =============================================================================
RUN apt-get update -y
RUN apt-get upgrade -y


# =============================================================================
# Install Debian Packages
# =============================================================================
RUN apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-plugins \
    git \
    python3-catkin-tools \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri


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
    -p $(openssl passwd -6 rosuser)
RUN echo "rosuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rosuser && chmod 440 /etc/sudoers.d/rosuser
WORKDIR /home/rosuser
USER rosuser


# =============================================================================
# Updating ROS Dependencies
# =============================================================================
RUN sudo rosdep update --rosdistro noetic
# RUN rosdep update --rosdistro noetic


# =============================================================================
# Clone + Build Simulation from GitHub Repo
# =============================================================================
WORKDIR /home/rosuser/sim_ws/src
# RUN git clone \
RUN sudo git clone \
    --recurse-submodules \
    https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
WORKDIR /home/rosuser/sim_ws
# RUN rosdep install \
RUN sudo rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro noetic \
    -y
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN sudo bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


# =============================================================================
# Set Default User
# =============================================================================
# USER rosuser


# =============================================================================
# End of File
# =============================================================================
