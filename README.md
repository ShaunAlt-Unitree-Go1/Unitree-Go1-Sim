# Unitree-Go1-Sim
Unitree Go1 Robot Full Simulation with LiDAR and Camera Support

## Table of Contents
- [Contributors](#contributors)
- [Supports](#supports)
- [From Scratch](#from-scratch)
- [Usage](#usage)
    - [Docker Usage](#docker-usage)
    - [Working with ROS2 Navigation (Single Robot without Namespace)](#working-with-ros2-navigation-single-robot-without-namespace)
    - [Working with ROS2 Navigation (Single Robot with Namespace)](#working-with-ros2-navigation-single-robot-with-namespace)
    - [Working with ROS2 Navigation (Multiple Robots)](#working-with-ros2-navigation-multiple-robots)

## Contributors
Created by: Shaun Altmann (saltmann@deakin.edu.au).

## Supports
ROS Humble on Ubuntu 22.04.
ROS Jazzy on Ubuntu 24.04.

## From Scratch
1. Create VM. In this case, I have downloaded the Ubuntu 24.04.1 ARM Server Image, and created a VM in Parallels giving it 8CPUs and 24GB of RAM. Go through the normal setup process, skipping all additional installation steps. This should end with you selecting to 'Reboot Now' the VM.
2. Install Ubuntu Desktop.
    ``` bash
    $ sudo apt update
    $ sudo apt upgrade -y
    $ sudo apt install ubuntu-desktop-minimal
    $ sudo reboot
    ```
3. Install ROS Jazzy.
    ``` bash
    $ sudo apt install software-properties-common
    $ sudo add-apt-repository universe
    $ sudo apt update
    $ sudo apt install curl -y
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    $ sudo apt update
    $ sudo apt install \
        ros-dev-tools \ # ros development tools
        ros-jazzy-desktop \ # ros
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup \ # nav
        ros-jazzy-slam-toolbox \ # slam
        ros-jazzy-ros-gz \ # gazebo
        -y
    $ sudo apt upgrade -y
    ```
4. Install Docker.
    ``` bash
    $ curl -fsSL https://get.docker.com/ | sh
    $ docker --version # validate docker is installed
    $ sudo groupadd docker
    $ sudo gpasswd -a $USER docker
    $ sudo systemctl restart docker
    $ xhost +local:
    ```
5. Update VM Hosts (used if connecting to Physical Robot).
    ``` bash
    $ sudo nano /etc/hosts
    # add the following line to the file
    192.168.123.161 raspberrypi
    ```
5. Update, Upgrade, and Restart VM.
    ``` bash
    $ sudo apt update
    $ sudo apt upgrade -y
    $ sudo reboot
    ```

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
1. Install Docker by following [these](https://docs.docker.com/engine/install/ubuntu/) steps.
2. Create and build this Docker.
    ``` bash
    # on main machine
    $ cd ~/
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/Unitree-Go1-Sim.git
    $ ./Unitree-Go1-Sim/docker/build_docker.bash
    ```
3. Create and build the ROS Bridge Docker.
    ``` bash
    # on main machine
    $ cd ~/
    $ git clone https://github.com/ShaunAlt-Unitree-Go1/ROS-Bridge-Docker.git
    $ ./ROS-Bridge-Docker/build_docker.bash
    ```
4. Install ROS2, Gazebo, SLAM, and Nav2 (using Humble in this example):
    ``` bash
    # on main machine
    $ cd ~/
    # follow steps from https://docs.ros.org/en/humble/Installation.html to install ros
    $ sudo apt install \
        ros-humble-ros-gz \ # gazebo
        ros-humble-navigation2 ros-humble-nav2-bringup \ # nav2
        ros-humble-slam-toolbox # slam
    ```
5. Run the ROS1 and ROS2 Nodes (open a new terminal for each of the following steps):
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
