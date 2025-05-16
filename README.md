# 2dslam-navigation-T265-with-hectorslam
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



# License
MIT License
