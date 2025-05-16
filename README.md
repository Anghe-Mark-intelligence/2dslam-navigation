# 2dslam-navigation-hectorslam-with-T265
A 2D SLAM based AGV Ackermann or Mecanum chassis navigation method that can be applied to ship clearance robots or automatic navigation robots
Contains multiple files, based on the ROS of Ubuntu 18.04 under Linux, which is a part of the workspace's src.
This project is for learning reference only and is convenient for personal use,the robots related to this project have been published in English journals.
In this paper, the hardware of our robot mainly consists of a controller module, radar module, power module, and visual recognition module. The system architecture framework of the robot is shown in Figure below.
Our controller mainly uses NUC11PAHi7 with 32GB of running memory, providing powerful computing power for Simultaneous Locali-zation and Mapping (SLAM) and visual algorithms, it can control the speed of the robot in real time by receiving information from the IMU and gyroscope to control the motor speed. We use Wheeltec Radar sensor, the Radiant M10P, located in the middle of the front of our robot and connected to the central controller through a USB serial port. The robotic arm on the robot is a simple bus connection method, equipped with six servos, and connected to the central controller via Ardunio Mini and micro interface via USB. We installed the Ubuntu 18.04 system on NUC and built upon it with Robot Operating System.For the Chinese version, please click [here](https://blog.csdn.net/qq_52529995/article/details/143632105?spm=1001.2014.3001.5502).
<div align="center">
  <img src="https://github.com/user-attachments/assets/5e3940cd-f5d2-4f6b-afd9-26e986ee74f2" alt="System Architecture" />
</div>

<div align="center"><b>Figure: System Architecture</b></div>


## Hardware Overview

- **Controller Module**: NUC11PAHi7 (32GB RAM), responsible for SLAM, visual algorithms, and robot control via IMU and gyroscope data.
- **Lidar Module**: Wheeltec M10P, mounted at the front center of the robot, connected to the controller via USB serial port.
- **Power Module**: Provides stable power supply for the robot and peripherals.
- **Visual Recognition Module**: Supports real-time image processing and object detection tasks.
- **Manipulator Arm**: 6-DOF servo-driven arm, connected through Arduino Mini and USB micro interface.
- **Operating System**: Ubuntu 18.04 with ROS (Robot Operating System) framework.
<p align="center">
  <img src="https://github.com/user-attachments/assets/08a78699-0f30-4a39-b25a-025b95593269" alt="UGV1" />
</p>
<p align="center"><b>Figure 1: Unmanned Ground Vehicle (UGV1)</b></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/6e535831-a10c-4e7c-b309-d15621b8f971" alt="UGV2" />
</p>
<p align="center"><b>Figure 2: Unmanned Ground Vehicle (UGV2)</b></p>



# Setting
Our robot requires ROS（Robot Operating system).To install ROS, you can refer to the following [tutorial](https://www.ros.org/).
```bash
mkdir angrobotarm
cd angrobotarm
conda create --name angrobot python=3.8
conda activate angrobot
sudo apt install ./packages/Markrobot_<version>_amd64.deb
```
Run the following command to validate the installation:
```bash
Mark_read_params -v
# Example output:
# 1.12
```
## Install Hector SLAM on Ubuntu 18.04 (ROS Melodic)

To perform 2D SLAM mapping with our UGV, we use the Hector SLAM package. Hector SLAM is a lightweight, laser-based SLAM algorithm that does not require odometry or IMU input, making it suitable for platforms with limited sensors.

### Installation Steps
```bash
# Step 1: Create a catkin workspace if not already created
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# Step 2: Clone the Hector SLAM source code into the workspace
cd ~/catkin_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git

# Step 3: Install required dependencies
sudo apt update
sudo apt install ros-melodic-hector-mapping ros-melodic-hector-nav-msgs ros-melodic-hector-trajectory-server ros-melodic-hector-geotiff ros-melodic-hector-geotiff-plugins

# Step 4: Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Step 5: Verify installation with a demo launch
roslaunch hector_slam_launch tutorial.launch
```
## Install Intel RealSense Viewer(to Visualize T265) on Ubuntu 18.04 (ROS Melodic)
### Installation Steps
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6B0FC61
sudo add-apt-repository "deb http://realsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update

sudo apt install ros-melodic-realsense2-camera

roslaunch realsense2_camera rs_t265.launch

rosrun rviz rviz
# In RViz, add "Odometry" display and select topic: /t265/odom/sample

```

# SLAM（Simultaneous Localization and Mapping）
To visualize SLAM mapping, you need to first start the car, connect the car's WiFi, SSH into the IP, and then open the automation program or manually operate it to create maps in a room with walls.

```bash
ssh 192.168.12.1@angrobot -y 12345678
rosrun rviz rviz
```
After SSH, operate the remote control to start SLAM mapping and visualize it in RVIZ
<p align="center">
  <img src="https://github.com/user-attachments/assets/2fbbdfe5-373d-4ae1-8435-c537e579d676" alt="Simultaneous Localization and Mapping" width="60%" />
</p>

<p align="center"><b>Figure: Simultaneous Localization and Mapping</b></p>




# Path planning
The main control program aecom_commander.cpp provides two path planning solutions: one that includes yaw rotation for directional adjustments, and another that performs path planning without yaw rotation.
The integrated SLAM algorithm can provide localization information for various mobile robots, such as unmanned ground vehicles (UGVs), unmanned aerial vehicles (UAVs), quadruped robots, and more. However, in this project, we focus only on our own UGV implementation for convenience. Other applications can be extended based on this framework.
The aecom_commander.cpp file is the main control program for the UGV, implementing a finite state machine (FSM) to manage navigation, path planning, and lifting operations. It provides two path planning methods: one with active yaw adjustment for precise orientation control, and another without yaw rotation for simpler linear movements. The UGV's position is continuously updated using tf transforms, while velocity commands are published to control motion. A serial communication interface handles the lifting mechanism. This design allows the UGV to autonomously follow a predefined path through a series of waypoints, transitioning between states based on real-time position feedback.

```bash
cd ugv_ros_ws
catkin build
```
After completion, please enter the following code：
```bash
roslaunch hector_slam heang_hector_ugv_shanghai-allinone.launch
```
Subsequently, a pile of information will pop up, and the appearance of real coordinates and predetermined coordinates print indicates success. The current case will continue to be printed until entering the next case, at which point we can press the button to start the task.

If you have not configured a channel, please set the automatic button on [qgroundcontrol](https://qgroundcontrol.com/) first. Only after turning the automatic button will the autonomous vehicle start moving.

<p align="center">
  <img src="https://github.com/user-attachments/assets/01d28008-ea6d-4395-ae69-fd4fc25ae3d4" alt="Path Planning for UGVs" width="60%" />
</p>

<p align="center"><b>Gif: Path Planning for UGVs</b></p>



# Obstacle avoidance
We using costmap_2d to avoid obstacles.The costmap_2d package is a core component of the ROS Navigation Stack, used to represent the environment around the robot for path planning and obstacle avoidance. It generates a 2D grid-based map where each cell holds a "cost" value that indicates how safe or risky it is for the robot to traverse that area.
## Install costmap_2d on Ubuntu 18.04 (ROS Melodic)
```bash
# Step 1: Install the ROS navigation stack, which includes costmap_2d
sudo apt update
sudo apt install ros-melodic-navigation

# Step 2: Verify that costmap_2d is available
roscd costmap_2d

# If you can enter the costmap_2d directory, the installation is successful.

# Step 3: (Optional) Clone the source code for customization
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/navigation.git

# Step 4: Build the workspace (if using from source)
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Step 5: Test a costmap demo (if provided) or integrate it in your local planner

```

<p align="center">
  <img src="https://github.com/user-attachments/assets/733d1360-0c74-4b23-a745-3c38bb670b56" alt="Autonomous Obstacle Avoidance for UGVs" />
</p>

<p align="center"><b>Gif: Autonomous Obstacle Avoidance for UGVs</b></p>






# State of UGV

By using status indicators and feedback data, we can define the states of the UGV (Unmanned Ground Vehicle) as follows:

| State                              | Color                                                        | Description                                                                                  |
|-------------------------------------|--------------------------------------------------------------|----------------------------------------------------------------------------------------------|
| Self-check                          | <span style="color: #ffcc00;">Breathing</span>                | The UGV is powered on and performing system self-check.                                       |
| Power On                            | <span style="color: #ffffff; background-color: #000000;">Constant White</span> | The UGV is powered on and ready to be controlled.                                              |
| Idle (Online)                       | <span style="color: #00ff00;">Constant Green</span>           | The UGV is online and standing by for navigation commands.                                     |
| Moving (Online)                     | <span style="color: #00ff00; font-weight: bold;">Flashing Green</span> | The UGV is actively moving according to received path planning commands.                      |
| Manual Control                      | <span style="color: #00ffff;">Constant Cyan</span>            | The UGV is under manual control mode for fine adjustments or calibration.                     |
| Lifting (Up/Down Operation)         | <span style="color: #0000ff;">Constant Blue</span>            | The UGV is performing lifting operations, adjusting platform height.                          |
| Trajectory Playback (Offline)       | <span style="color: #800080;">Constant Purple</span>         | The UGV is replaying a recorded trajectory in offline mode. External commands are ignored.    |
| Moving to Starting Point (Offline)  | <span style="color: #800080; font-weight: bold;">Flowing Purple</span> | The UGV is moving to the starting point of a recorded trajectory in offline mode.             |
| Executing Playback (Offline)        | <span style="color: #800080; font-weight: bold;">Breathing Purple</span> | The UGV is executing a trajectory replay autonomously.                                       |
| Error                               | <span style="color: #ff0000;">Constant Red</span>            | The UGV has encountered an error state and has stopped movement.                              |

> The states of the UGV can be switched through control commands from ROS nodes or via physical control buttons.


# UAV
We have also developed a platform for drones, including PX4, radar, LiDAR, and T265. Please refer to the following for [details](https://blog.csdn.net/qq_52529995/article/details/143703573?spm=1001.2014.3001.5502).
<p align="center">
  <img src="https://github.com/user-attachments/assets/b3dcdcd7-4ec3-45df-b369-3dc188391a7f" alt="UAV Diagram" width="60%" />
</p>
<p align="center"><b>Figure: Unmanned Aerial Vehicle (UAV)</b></p>

# Citation
If you find the code useful, please cite:


# Note
This is the first project I was responsible for during my undergraduate studies. As the team leader, I independently developed UAV and UGV systems and won awards for unmanned aerial vehicles in the International Unmanned Systems Competition and the second prize in the China Robotics Competition. The submission and publication of the paper were quite bumpy, with several rejections and a long time span. This readme mainly describes how unmanned systems (unmanned vehicles, unmanned aerial vehicles) are used, and there may be some errors in the specific replication process, such as interfaces, protocols, or installation programs. As time has passed for a long time, the unmanned systems mentioned in the article are rarely used after the project ends. Therefore, if there are any problems, please feel free to contact my email: ANGHE2004MARK@outlook.com I am happy to discuss with you issues related to unmanned vehicles, drones, path planning, forward and inverse kinematics of robotic arms, and more.

# License
MIT License
