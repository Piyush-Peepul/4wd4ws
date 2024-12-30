# ROS2 RSBL Driver

## Overview
The **ros2_rsbl_driver** package is a ROS 2 (Robot Operating System 2) package designed to interface with and control RSBL (Robot Servo Bus Line) devices. This package provides functionality for calibrating servo offsets and controlling motors through configurable parameters and launch files.

## Features
- **Servo Calibration:** Tools for calibrating servo offsets.
- **Motor Control:** Interface for controlling RSBL motors.
- **Configurable Parameters:** YAML-based configuration for customization.
- **Launch Files:** Predefined launch files for easy setup and execution.

## Package Contents

### Key Files and Directories

- **`package.xml`**: Defines the package metadata and dependencies.
- **`CMakeLists.txt`**: Build configuration for the package.
- **`config/`**: Contains configuration files for the driver.
  - `rsbl_calibrate.yaml`: Calibration-specific configuration.
  - `rsbl_driver_config.yaml`: General driver configuration.
- **`include/`**: Header files for the package.
  - `ros2_rsbl_driver/`: Main headers for the package.
  - `SCServo_Linux/`: Library headers for SCServo communication.
- **`launch/`**: Contains launch files for starting nodes.
  - `callibrate_servo_offset.launch.py`: Launch file for servo offset calibration.
  - `rsbl_driver.launch.py`: Launch file for the RSBL driver.
- **`src/`**: Source code for the package.
  - `calibrate_offset.cpp`: Node for servo calibration.
  - `ros2_rsbl_driver.cpp`: Main driver node.
  - `test_motors.cpp`: Node for testing motor functionality.

## Dependencies
This package depends on the following ROS 2 packages:
- `rclcpp`
- `std_msgs`

## Installation

1. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Build the package:
   ```bash
   colcon build --symlink-install --packages-select ros2_rsbl_driver
   ```

3. Source the workspace & underlay:
   ```bash
   source ~/ros2_ws/install/setup.bash
   source /opt/ros/<ROS_DISTRO>/setup.bash
   ```

## Usage

### Launch the RSBL Driver
To launch the RSBL driver with default configurations:
```bash
ros2 launch ros2_rsbl_driver rsbl_driver.launch.py
```

### Calibrate Servo Offset
To calibrate servo offsets:
```bash
ros2 launch ros2_rsbl_driver callibrate_servo_offset.launch.py
```

### Configuration
Modify the YAML configuration files in the `config/` directory to customize the behavior of the nodes:
- **`rsbl_calibrate.yaml`**: Adjust calibration parameters.
- **`rsbl_driver_config.yaml`**: Set motor and driver-specific settings.

## Source
https://www.waveshare.com/wiki/RSBL45-24

## Maintainer
- Piyush Mahamuni (<piyush.mahamuni@peepulagri.com>)
