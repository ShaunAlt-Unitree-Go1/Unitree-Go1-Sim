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
### Docker Usage
1. Clone this repository.
    ``` bash
    $ cd ~
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
    ```
2. Create the Docker Image.
    ``` bash
    $ ./docker/build_docker.sh
    ```
3. Run the Docker Image.
    ``` bash
    $ ./docker/run_docker.sh
    # use the -h option to see helper info on this script
    ```
4. If you want to open another terminal inside the docker, open a new terminal and run the following command:
    ``` bash
    $ ./docker/run_docker.bash -t
    ```
5. To get the simulation working inside the docker, run the Docker and use the following commands:
    ``` bash
    $ cd /home/rosuser/sim_ws # you should already be here
    $ source devel/setup.bash
    $ roslaunch go1_simulation go1_simulation.launch
6. If the above `roslaunch` command fails, try just opening the world using gazebo first, then reattempting the `go1_simulation` launch.
    ``` bash
    $ roslaunch gazebo_ros willowgarage_world.launch --screen
    ```
7. Once the simulation is working, you can open RVIZ in a new terminal (inside the docker) to use the SLAM and Navigation Stacks.
    ``` bash
    $ source /home/rosuser/sim_ws/devel/setup.bash
    $ roslaunch go1_simulation slam.launch rviz:=true
    ```

### Docker + ROS Bridge + ROS2 Nav
1. Create and build this Docker.
    - Open the docker and run the following:
        ``` bash
        $ source devel/setup.bash
        $ roslaunch gazebo_ros willowgarage_world.launch # CTRL+C once it loads
        $ roslaunch go1_simulation go1_simulation.launch # leave this running
        ```
2. Create and build the ROS bridge Docker.
    - Open the docker and run the following:
        ``` bash
        $ source install/setup.bash
        $ ros2 run ros1_bridge dynamic_bridge # leave this running
        ```
3. Install ROS Navigation on main machine.
    ``` bash
    $ sudo apt install ros2-humble-navigation2 ros2-humble-nav2-bringup
    ```
4. Open 5 terminals, and run the following in each (in order):
    - Use this to setup the `base_link` frame required by Nav2.
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run tf2_ros static_transform_publisher --frame-id trunk --child-frame-id base_link
        ```
    - Use this to setup the `odom` frame required by Nav2.
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_footprint
        ```
    - Use this to run Nav2.
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run nav2_bringup navigation_launch.py use_sim_time:=true
        ```
    - Use this to run SLAM.
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run slam_toolbox online_async_launch.py use_sim_time:=true
        ```
    - Use this to test the transforms + run RViZ to see the map.
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run tf2_tools view_frames # will generate a pdf of the frames map
        $ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
        ```
5. Fully Working ROS2 (7 terminals):
    1. Unitree Go1 Sim Docker (ROS Noetic).
        ``` bash
        $ source devel/setup.bash
        $ roslaunch gazebo_ros willowgarage_world.launch
        $ roslaunch go1_simulation go1_simulation.launch
        ```
    2. Unitree Go2 Sim Docker (ROS Noetic).
        ``` bash
        $ source devel/setup.bash
        $ roslaunch go1_simulation slam.launch
        ```
    3. ROS Bridge Docker (ROS Galactic).
        ``` bash
        $ source install/setup.bash
        $ ros2 run ros1_bridge dynamic_bridge
        ```
    4. Main Machine (ROS Humble).
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run tf2_ros static_transform_publisher --frame-id trunk --child-frame-id base_link
        ```
    5. Main Machine (ROS Humble).
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run nav2_bringup navigation_launch.py use_sim_time:=true
        ```
    6. Main Machine (ROS Humble).
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run slam_toolbox online_async_launch.py use_sim_time:=true
        ```
    7. Main Machine (ROS Humble).
        ``` bash
        $ source /opt/ros/humble/setup.bash
        $ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
        ```

### Using in a VM
> [!NOTE]
> `/move_base_simple/goal` is used by RVIZ when using the "2D Nav Goal" feature to make the robot go to a particular location.

See https://github.com/Taucrates/simulation_unitree_go1.
1. Create a new workspace and source ROS Noetic.
    ``` bash
    $ source /opt/ros/noetic/setup.sh
    $ mkdir -p ~/noetic_sim_ws/src
    $ cd ~/noetic_sim_ws/src
    ```
2. Import this repo (and all dependencies).
    ``` bash
    $ git clone --recurse-submodules https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
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

### Trying to Work Out Their SLAM
- SLAM Launch `go1_simulation -> slam.launch`
    - INFO:
    - CALLS:
        - NODE `laser_scan_matcher.laser_scan_matcher_node`
        - INCLUDE `go1_config/launch/include/gmapping.launch`
            - References the `base_footprint`, `odom`, and `map` frames.
        - INCLUDE `go1_config/launch/include/move_base.launch`

