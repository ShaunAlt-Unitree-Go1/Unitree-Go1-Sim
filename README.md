# Unitree-Go1-Sim
Unitree Go1 Robot Full Simulation with LiDAR and Camera Support

## Table of Contents
- [Contributors](#contributors)
- [Supports](#supports)
- [Usage](#usage)
    - [Docker Usage](#docker-usage)
    - [Working with ROS2 Navigation (Single Robot without Namespace)](#working-with-ros2-navigation-single-robot-without-namespace)
    - [Working with ROS2 Navigation (Single Robot with Namespace)](#working-with-ros2-navigation-single-robot-with-namespace)
    - [Working with ROS2 Navigation (Multiple Robots)](#working-with-ros2-navigation-multiple-robots)

## Contributors
Created by: Shaun Altmann (saltmann@deakin.edu.au).

## Supports
ROS Noetic on Ubuntu 20.04.

## Usage
### Docker Usage
1. Clone this repository.
    ``` bash
    $ cd ~/
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
    ```
2. Create the Docker Image.
    ``` bash
    $ cd ~/
    $ ./Unitree-Go1-Sim/docker/build_docker.bash
    ```
3. Run the Docker Image.
    ``` bash
    $ ./docker/run_docker.bash
    # use the -h option to see helper info on this script
    ```
4. If you want to open another terminal inside the docker, open a new terminal and run the following command:
    ``` bash
    $ ./docker/run_docker.bash -t
    ```

### Working with ROS2 Navigation (Single Robot without Namespace)
1. Create and build this Docker.
    ``` bash
    # on main machine
    $ cd ~/
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
    $ ./Unitree-Go1-Sim/docker/build_docker.bash
    ```
2. Create and build the ROS Bridge Docker.
    ``` bash
    # on main machine
    $ cd ~/
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Docker.git
    $ ./ROS-Bridge-Docker/build_docker.bash
    ```
3. Install ROS2, SLAM, and Nav2 (using Humble in this example):
    ``` bash
    # on main machine
    $ cd ~/
    # follow steps from https://docs.ros.org/en/humble/Installation.html to install ros
    $ sudo apt install ros2-humble-navigation2 ros2-humble-nav2-bringup
    ```
4. Run the ROS1 and ROS2 Nodes (open a new terminal for each of the following steps):
    1. Run the Go1 Simulation.
        ``` bash
        # on main machine
        $ cd ~/
        $ ./Unitree-Go1-Sim/docker/run_docker.bash
        # now inside the simulation docker
        $ cd /home/rosuser/sim_ws/
        $ source devel/setup.bash
        $ roslaunch gazebo_ros willowgarage_world.launch
        # hit CTRL+C once gazebo loads this world
        $ roslaunch go1_simulation go1_simulation.launch
        ```
    2. Run the Go1 Lidar Node.
        ``` bash
        # on main machine
        $ cd ~/
        $ ./Unitree-Go1-Sim/docker/run_docker.bash -t
        # now inside the simulation docker
        $ cd /home/rosuser/sim_ws/
        $ source devel/setup.bash
        $ roslaunch go1_simulation slam.launch
        ```
    3. Run the ROS Bridge.
        ``` bash
        # on main machine
        $ cd ~/
        $ ./ROS-Bridge-Docker/run_docker.bash
        # now inside the bridge docker
        $ cd /home/rosuser/bridge_ws/
        $ source install/setup.bash
        $ rosparam load bridge.yaml
        $ ros2 run ros1_bridge parameter_bridge
        ```
    4. Run a Static Transform Publisher.
        ``` bash
        # on main machine
        $ source /opt/ros/humble/setup.bash
        $ ros2 run tf2_ros static_transform_publisher --frame-id trunk --child-frame-id base_link
        ```
    5. Run the ROS2 Navigation Stack.
        ``` bash
        # on main machine
        $ source /opt/ros/humble/setup.bash
        $ ros2 run nav2_bringup navigation_launch.py use_sim_time:=true
        ```
    6. Run the ROS2 SLAM Node.
        ``` bash
        # on main machine
        $ source /opt/ros/humble/setup.bash
        $ ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
        ```
    7. Run RViZ2 to visualize and use the navigation stack.
        ``` bash
        # on main machine
        $ source /opt/ros/humble/setup.bash
        $ ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
        ```
    8. (OPTIONAL) View the transform frames.
        ``` bash
        # on main machine
        $ cd ~/
        $ source /opt/ros/humble/setup.bash
        $ ros2 run tf2_tools view_frames
        # this will generate a .pdf document in the `~/` directory
        ```

### Working with ROS2 Navigation (Single Robot with Namespace)
> [!NOTE]
> Currently working on getting this operational.

### Working with ROS2 Navigation (Multiple Robots)
> [!NOTE]
> Currently working on getting this operational.
