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
ENV LIBGL_ALWAYS_SOFTWARE=1


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
WORKDIR /home/rosuser


# =============================================================================
# Updating ROS Dependencies
# =============================================================================
RUN rosdep update --rosdistro noetic


# =============================================================================
# Clone + Build Simulation from GitHub Repo
# =============================================================================
WORKDIR /home/rosuser/sim_ws/src
RUN git clone \
    --recurse-submodules \
    https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git

# customize the `slam.launch` file - this way the lidar mapper is created, but
#  none of the other slam gmapping or move_group stuff is created
RUN rm -rf /home/rosuser/sim_ws/src/Unitree-Go1-Sim/submodules/Taucrates/simulation_unitree_go1/go1_simulation/launch/slam.launch
RUN rm -rf /home/rosuser/sim_ws/src/Unitree-Go1-Sim/submodules/Taucrates/simulation_unitree_go1/go1_description/xacro/go1.urdf.xacro
RUN rm -rf /home/rosuser/sim_ws/src/Unitree-Go1-Sim/submodules/Taucrates/simulation_unitree_go1/go1_description/urdf/go1.urdf
COPY custom/slam.launch /home/rosuser/sim_ws/src/Unitree-Go1-Sim/submodules/Taucrates/simulation_unitree_go1/go1_simulation/launch/
COPY custom/go1.urdf.xacro /home/rosuser/sim_ws/src/Unitree-Go1-Sim/submodules/Taucrates/simulation_unitree_go1/go1_description/xacro/go1.urdf.xacro
COPY custom/go1.urdf /home/rosuser/sim_ws/src/Unitree-Go1-Sim/submodules/Taucrates/simulation_unitree_go1/go1_description/urdf/go1.urdf

WORKDIR /home/rosuser/sim_ws
RUN rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro noetic \
    -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


# =============================================================================
# Add Gazebo Worlds
# =============================================================================
WORKDIR /home/rosuser/
RUN git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
RUN bash -c 'echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/rosuser/gazebo_models_worlds_collection/models" >> /home/rosuser/.bashrc'
RUN bash -c 'echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/rosuser/gazebo_models_worlds_collection/worlds" >> /home/rosuser/.bashrc'


# =============================================================================
# Set Default User
# =============================================================================
WORKDIR /home/rosuser/
USER rosuser


# =============================================================================
# End of File
# =============================================================================
