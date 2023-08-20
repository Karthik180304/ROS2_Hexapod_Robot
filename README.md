# Hexapod Robot Line Following Project

This repository contains a ROS2-based project for a hexapod robot with line-following functionality. The project consists of three packages: `articubot_one`, `my_robot_bringup`, and `hexapod_robot`. Each package serves a specific purpose in the overall system.

![image](https://github.com/Karthik180304/ROS2_Hexapod_Robot/assets/118727786/73a43ad4-a82d-4f25-988b-e2b991d39699)


## Packages

### articubot_one

The `articubot_one` package contains the URDF files and a launch file to run the hexapod robot model in simulation. This package is responsible for defining the physical structure of the robot and configuring the launch settings for simulation.

### my_robot_bringup

The `my_robot_bringup` package provides a launch file that allows you to run the Python nodes from the `hexapod_robot` package simultaneously. This package handles the orchestration of the different components of the hexapod robot, enabling you to start all necessary nodes conveniently.

### hexapod_robot

The `hexapod_robot` package contains the Python node files that define the kinematics of the hexapod robot, implement the line following functionality, and provide a walking movement code. This package forms the core of the project, implementing the robot's behavior and control logic.

## Getting Started

To use this project, follow the steps below:

1. Install ROS2 (insert ROS2 installation instructions here).
1.2 Create a ROS2 workspace:

   ```
   mkdir -p ~/ros2_workspace/src
   cd ~/ros2_workspace/src
   ```
   
2. Clone this repository:

   ```
   git clone git clone https://github.com/Karthik180304/ROS2_Hexapod_Robot.git
   ````

1. Build the project using colcon:
    ```
    cd ~/ros2_workspace
    colcon build
    ```
2. Source the setup files:

    ```
    source install/setup.bash
    ```

3. Launch the simulation:
    ```
    ros2 launch articubot_one rsp.launch.py
    ```
This command will launch the hexapod robot model in Gazebo, with the line following and walking functionality implemented.


## Contributing

Contributions to this project are welcome! If you have any ideas, improvements, or bug fixes, feel free to submit a pull request. Please ensure that your changes align with the project's coding style and standards.
