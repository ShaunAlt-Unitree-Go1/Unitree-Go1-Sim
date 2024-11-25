# Unitree-Go1-Sim
Unitree Go1 Robot Full Simulation with LiDAR and Camera Support

## Table of Contents
- [Contributors](#contributors)
- [Supports](#supports)
- [Usage](#usage)

## Contributors
Created by: Shaun Altmann (saltmann@deakin.edu.au).

## Supports
ROS Noetic on Ubuntu 20.04.

## Usage
See https://github.com/Taucrates/simulation_unitree_go1.
1. Create a new workspace and source ROS Noetic.
    ``` bash
    $ source /opt/ros/noetic/setup.sh
    $ mkdir -p ~/noetic_sim_ws/src
    $ cd ~/noetic_sim_ws/src
    ```
2. Import this repo (and all dependencies).
    ``` bash
    $ git clone --recurse-submodules https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim
    ```
3. Go back to workspace, install all ROS dependencies.
    ``` bash
    $ cd ~/noetic_sim_ws
    $ rosdep update --rosdistro noetic
    $ rosdep install --from-paths src --ignore-src --rosdistro noetic -y
    ```
4. Build the workspace.
    ``` bash
    $ catkin_make
    ```
5. Source the workspace.
    ``` bash
    $ source devel/setup.sh
    ```
6. Launch the simulation + SLAM/Nav Stack.
    - Terminal 1 (Simulation):
        ``` bash
        $ source ~/noetic_sim_ws/devel/setup.sh
        $ roslaunch go1_simulation go1_simulation.launch
        ```
    - Terminal 2 (SLAM / Nav Stack):
        ``` bash
        $ source ~/noetic_sim_ws/devel/setup.sh
        $ roslaunch go1_simulation slam.launch rviz:=true
        ```