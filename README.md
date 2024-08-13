# Moving UR5 in Gazebo Using Cartesian Coordinates

This repository provides the necessary files and instructions to move the UR5 robot in a Gazebo simulation environment using Cartesian coordinates. Follow the steps below to set up, run, and manage the simulation.

## Prerequisites

1. **ROS Installation**: Ensure you have [ROS](http://wiki.ros.org/ROS/Installation) installed on your system.
2. **Universal Robots ROS Driver**: Make sure the `universal_robot` package is installed. If not, you can clone it from the [universal_robot GitHub repository](https://github.com/ros-industrial/universal_robot) and build it in your catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/ros-industrial/universal_robot.git
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

3. **TRAC-IK Solver**: The TRAC-IK package is used for inverse kinematics. Install it using:

    ```bash
    sudo apt-get install ros-<your_ros_distro>-trac-ik
    ```

    Replace `<your_ros_distro>` with your ROS distribution name (e.g., `noetic`, `melodic`).

4. **Gazebo**: Ensure that Gazebo is installed. It's typically included with standard ROS installations.

## Setting Up the Environment

Launch the UR5 robot in Gazebo using the following command:

```bash
roslaunch ur_gazebo ur5_bringup.launch
```

This command initializes the Gazebo environment with the UR5 robot model, allowing for interaction through ROS.

## Running the Control Script

1. **Build the Workspace**: Before running the script, ensure your workspace is built:

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

2. **Launch Gazebo Simulation**: If not already running, start the Gazebo simulation:

    ```bash
    roslaunch ur_gazebo ur5_bringup.launch
    ```

3. **Run the Control Script**: In a new terminal, execute the control script:

    ```bash
    rosrun control_ur5 gazebo_ur5_hitUAV.py
    ```

   Upon execution, the UR5 robot in the Gazebo simulation should move through the specified Cartesian coordinates.

## Resetting the Simulation Environment

Before starting a new simulation run (especially after completing one), it's essential to reset the simulation environment to avoid conflicts or unintended behavior. Follow these steps:

1. **Terminate Existing Nodes**: First, ensure all ROS nodes are terminated. You can use:

    ```bash
    rosnode kill -a
    ```

2. **Kill Gazebo Processes**: Sometimes, Gazebo processes might remain active. Kill them using:

    ```bash
    killall -9 gzserver gzclient
    ```

3. **Cleanup ROS Parameters**:

    ```bash
    rosparam delete -y /
    ```

4. **Launch Gazebo Simulation Again**:

    ```bash
    roslaunch ur_gazebo ur5_bringup.launch
    ```

5. **Run the Control Script**:

    ```bash
    rosrun control_ur5 gazebo_ur5_hitUAV.py
    ```

**Note**: Always ensure that the simulation environment is clean before starting a new run to prevent any residual effects from previous simulations.

## Troubleshooting

- **IK Solution Not Found**: If the script logs errors about the IK solution not being found, ensure that the TRAC-IK solver is correctly installed and that the specified poses are within the robot's reachable workspace.

- **Simulation Performance Issues**: Running Gazebo can be resource-intensive. Ensure your system meets the recommended specifications for running Gazebo smoothly.

- **ROS Environment Variables**: If you encounter issues related to ROS environment variables, source your workspace:

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

## Acknowledgments

- [Universal Robots ROS Driver](https://github.com/ros-industrial/universal_robot)
- [TRAC-IK Solver](https://bitbucket.org/traclabs/trac_ik/src/master/)
- [ROS (Robot Operating System)](http://www.ros.org/)
