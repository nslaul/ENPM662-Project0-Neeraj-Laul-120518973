# TurtleBot Open Loop Controllers

This repository provides implementations of simple open loop controllers for controlling a TurtleBot in two specific scenarios using ROS 2. The controllers are designed to demonstrate basic motion control by commanding the robot to move at a constant velocity and to perform acceleration and deceleration maneuvers.

## Scenarios

### Scenario 1: Constant Velocity for 15 Seconds
- **Description**: The TurtleBot will move at a steady speed for 15 seconds.
- **Parameters**: You can configure the desired velocity in the respective code files.

### Scenario 2: Acceleration, Constant Velocity, and Deceleration
- **Description**: The TurtleBot will:
  - Accelerate for 5 seconds
  - Move at a constant velocity for 5 seconds
  - Decelerate for 5 seconds
- **Parameters**: You can adjust the acceleration, maximum velocity, and deceleration rates in the respective code files.

## Dependencies

To run this project, ensure you have the following dependencies installed:

- **ROS 2** (Humble, but can work on Foxy or later with a few minor changes)
- **turtlebot3**: Install the TurtleBot3 packages - can be found by using "wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos" in your "src" folder or by following instructions at the ROBOTIS website.

### Installation

1. **Install ROS 2**:
   Follow the instructions on the [ROS 2 installation page](https://docs.ros.org/en/humble/Installation/).

2. **Install TurtleBot3 packages**:
   ```bash
   sudo pip install vcstool
   vcs import src<turtlebot3.repos
   ```

3. **Clone the repository**:
   ```bash
   git clone https://github.com/nslaul/ENPM662-Project0-Neeraj-Laul-120518973
   ```

4. **Check for missing dependancies and packages**:
   ```bash
   rosdep check --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   ```

5. **Install missing packages**:
   ```bash
   sudo apt-get install <Your_Package_Name>
   ```
   **Or use aptitude**:
   ```bash
   sudo apt-get install aptitude
   sudo aptitude install <Missing_Package_Name>
   ```
   Continue using the instructions on screen and go through all of the missing packages until
   ```bash
   rosdep check --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   ```
   Displays "All system dependancies satisfied".

6. **Build the package**:
   ```bash
   colcon build
   ```

6. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launching the Controllers

To run the open loop controllers, do the following:
1. Ensure you are in the project directory that has been built and sourced.
2. **Open Gazebo with the Turtlebot in an empty world**:
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

1. **Scenario 1**: Constant Velocity
   ```bash
   ros2 run tb_control tb_scenario1
   ```

2. **Scenario 2**: Acceleration, Constant Velocity, and Deceleration
   ```bash
   ros2 run tb_control tb_scenario2
   ```

## Troubleshooting

- Ensure that your workspace folder builds correctly, follow the steps for building to troubleshoot any missing packages.
- It is recommended to have the repository in the "src" folder of your workspace folder to avoid any file pathing issues.

## Acknowledgments

- Special thanks to the ROS 2 community and the TurtleBot development team for their ongoing support and contributions.

For further questions or contributions, feel free to open an issue or submit a pull request!
