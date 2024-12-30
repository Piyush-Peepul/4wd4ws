# ROS2 DDSM115 Driver

## Overview
The **ros2_ddsm115_driver** package is a ROS 2 (Robot Operating System 2) package designed to interface with and control ddsm115 hub motor devices. This package provides functionality for controlling motors through configurable parameters and launch files.

## Features
- **Motor Control:** Interface for controlling ddsm115 hub motors.
- **Configurable Parameters:** YAML-based configuration for customization.
- **Launch Files:** Predefined launch files for easy setup and execution.

## Package Contents

### Key Files and Directories

- **`package.xml`**: Defines the package metadata and dependencies.
- **`CMakeLists.txt`**: Build configuration for the package.
- **`config/`**: Contains configuration files for the driver.
  - `ddsm115_driver_config.yaml`: General driver configuration.
- **`launch/`**: Contains launch files for starting nodes.
  - `ddsm115_driver.launch.py`: Launches the driver node with above mentioned configuration file.
- **`src/`**: Source code for the package.
  - `ddsm115_communication.hpp`: Header for serial communication with ddsm115 motors.
  - `ddsm115_communication.cpp`: Implementation of the same.
  - `ros2_ddsm115_driver.hpp`: Header & implementation of derived ros2 node class as a wrapper on top of ddsm115_comminucation.
  - `ros2_ddsm115_driver.cpp`: ros2 driver node

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
   colcon build --symlink-install --packages-select ros2_ddsm115_driver
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   source /opt/<ROS_DISTRO>/setup.bash
   ```

## Usage

### Launch the DDSM115 driver
To launch the ddsm115 driver with default configurations:
```bash
ros2 launch ros2_ddsm_driver ddsm115_driver.launch.py
```

### Configuration
Modify the YAML configuration files in the `config/` directory to customize the behavior of the nodes:
- **`ddsm115_driver_config.yaml`**: Set motor and driver-specific settings.

## Problems Faced

### Voltage Levels
Although data sheet mentions Input voltage range to be 12V-24V, tests yielded input voltage above 17V to induce noise in the bus lines.
Also, anywhere below 20V, the hub motor drive fails short to match the input velocity.

## Source
https://www.waveshare.com/wiki/DDSM115 

## Maintainer
- Piyush Mahamuni (<piyush.mahamuni@peepulagri.com>)
