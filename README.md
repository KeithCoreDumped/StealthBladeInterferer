## Stealth Blade Interferer

### Project Overview

Stealth Blade Interferer(隐锋干扰者) is an autonomous driving car project that integrates multiple advanced systems, including LiDAR SLAM for navigation, YOLOv8 for object detection, and a Software-Defined Radio (SDR) jammer controlled by the car. The project employs two Raspberry Pi boards on the vehicle, along with two USRP devices deployed along the vehicle's path to simulate a wireless communication system for the car to interfere with. When the car detects the "xiaoba" object, it automatically triggers the SDR jammer.

### System Components

1. LiDAR SLAM and Navigation:
    - LiDAR Model: Livox Mid-360, using the Livox ROS driver.
    - SLAM Algorithm: FAST-LIO2 (LiDAR-Inertial Odometry).
    - Platform: ROS Noetic.
    - Raspberry Pi 1: Handles SLAM, navigation, motor control, and manages the USRP device used as the jammer.
2. Object Detection (YOLOv8):
    - Raspberry Pi 2: Dedicated to running YOLOv8 for object detection due to its high computational demand.
    - Primary Goal: Continuously detects the "xiaoba" object in real-time.
3. Software-Defined Radio (SDR) Jammer:
    - Hardware: USRP device mounted on the car.
    - Control: Managed by Raspberry Pi 1.
    - Functionality: The vehicle interferes with a simple wireless communication system when it detects the "xiaoba" object.
4. Ground-Side Wireless Communication System:
    - USRP Devices: Two USRP devices are placed near the vehicle’s path.
    - Computer Interface: A computer connects both USRP devices to simulate a wireless communication system. The vehicle disrupts this communication once it detects the object.

### System Architecture

- Raspberry Pi 1: Handles navigation, motor control, LiDAR SLAM, and the USRP SDR for the radio jammer.
- Raspberry Pi 2: Runs the YOLOv8 object detection model.
- Ground-Side USRP: The two USRP devices simulate the wireless communication system that the car disrupts once it detects the "xiaoba" object.

### Setup Instructions

1. Raspberry Pi 1 Setup:
    - Install ROS Noetic, Livox ROS driver, and FAST-LIO2 for LiDAR SLAM.
    - Configure motor control and integrate the USRP SDR as a jammer.
2. Raspberry Pi 2 Setup:
    - Install and configure YOLOv8 for object detection.
3. Ground-Side Setup:
    - Connect the two USRP devices to a computer.
    - Set up the wireless communication system for the vehicle to interfere with.

### Project Directory Structure

This project has two main components: the `Car` folder for navigation and object detection, and the `USRP` folder for communication and jamming.
### `Car/`
- `object-detection/`:
    - Contains YOLOv8 model data (`train/`, `test/`, `valid/`) and trained weights under `runs/detect/`.
- `ws_livox/src/`:
    - `chassis/`: Controls motors and chassis operations.
    - `FAST_LIO/` and `FAST_LIO_LOCALIZATION/`: SLAM and localization algorithms.
    - `livox_ros_driver2/`: LiDAR driver for Livox Mid-360.
    - `donkey_nav/`: Autonomous navigation scripts and launch files.

> For your reference, the pointcloud file used in the demonstration is located at `Car/ws_livox/src/FAST_LIO/PCD/jiao3map3/scans.pcd` 

### `USRP/`
- `communication/`: GRC files (`communicate1.grc` and `communicate2.grc`) configure the SDR for communication.
- `jammer/usrp_test1/`: Jamming system code and build files for the SDR jammer.