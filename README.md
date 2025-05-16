# 2dslam-navigation-hectorslam-with-T265
A 2D SLAM based AGV Ackermann or Mecanum chassis navigation method that can be applied to ship clearance robots or automatic navigation robots
Contains multiple files, based on the ROS of Ubuntu 18.04 under Linux, which is a part of the workspace's src.
This project is for learning reference only and is convenient for personal use,the robots related to this project have been published in English journals.
In this paper, the hardware of our robot mainly consists of a controller module, radar module, power module, and visual recognition module. The appearance of the robot is shown in Figure 2 and 3,  the system architecture framework of the robot is shown in Figure 1.
Our controller mainly uses NUC11PAHi7 with 32GB of running memory, providing powerful computing power for Simultaneous Locali-zation and Mapping (SLAM) and visual algorithms, it can control the speed of the robot in real time by receiving information from the IMU and gyroscope to control the motor speed. We use Wheeltec Radar sensor, the Radiant M10P, located in the middle of the front of our robot and connected to the central controller through a USB serial port. The robotic arm on the robot is a simple bus connection method, equipped with six servos, and connected to the central controller via Ardunio Mini and micro interface via USB. We installed the Ubuntu 18.04 system on NUC and built upon it with Robot Operating System.For the Chinese version, please click [here](https://blog.csdn.net/qq_52529995/article/details/143632105?spm=1001.2014.3001.5502).
<div align="center">
  <img src="https://github.com/user-attachments/assets/5e3940cd-f5d2-4f6b-afd9-26e986ee74f2" />

</div>

## Hardware Overview

- **Controller Module**: NUC11PAHi7 (32GB RAM), responsible for SLAM, visual algorithms, and robot control via IMU and gyroscope data.
- **Lidar Module**: Wheeltec M10P, mounted at the front center of the robot, connected to the controller via USB serial port.
- **Power Module**: Provides stable power supply for the robot and peripherals.
- **Visual Recognition Module**: Supports real-time image processing and object detection tasks.
- **Manipulator Arm**: 6-DOF servo-driven arm, connected through Arduino Mini and USB micro interface.
- **Operating System**: Ubuntu 18.04 with ROS (Robot Operating System) framework.
<p align="center">
  <img src="https://github.com/user-attachments/assets/08a78699-0f30-4a39-b25a-025b95593269" alt="image1" />
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/6e535831-a10c-4e7c-b309-d15621b8f971" alt="image2" />
</p>



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



# UAV
We have also developed a platform for drones, including PX4, radar, LiDAR, and T265. Please refer to the following for [details](https://blog.csdn.net/qq_52529995/article/details/143703573?spm=1001.2014.3001.5502).
<p align="center">
  <img src="https://github.com/user-attachments/assets/5564b9fb-9867-424b-985e-72b86e7a3f23" alt="UGV data" />
</p>



# License
MIT License
