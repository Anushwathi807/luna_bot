# LunaBot: Autonomous Lunar Habitat Rover 🌕🤖

This repository contains the source code for LunaBot, a ROS 2 simulation project for the Smart India Hackathon. The goal is to develop an autonomous 4-wheel rover capable of navigating, mapping, and monitoring a simulated lunar habitat to ensure astronaut safety and operational reliability.



## Core Technologies
* **ROS 2 Humble:** The core robotics framework.
* **Gazebo Simulator:** For creating and running the 3D simulation.
* **Nav2 Stack:** For autonomous navigation, including path planning and obstacle avoidance.
* **SLAM Toolbox:** For creating a map of the environment.
* **RViz:** For visualization and debugging.
* **Python:** For the high-level mission control script.

## Project Scope
The project aims to deliver the following key features in a simulation environment:
1.  A 4-wheeled rover model described in URDF.
2.  A simulated lunar habitat world in Gazebo with lunar gravity.
3.  **Autonomous Mapping:** The rover can be manually driven to generate a 2D map of its environment.
4.  **Autonomous Navigation:** The rover can autonomously navigate to a given goal on the map, avoiding obstacles.
5.  **Mission Execution:** The rover can execute a pre-defined patrol route while monitoring simulated environmental data (e.g., oxygen levels) and signaling an alert if an anomaly is detected.

---

## Step-by-Step Implementation Guide

### Phase 0: Setup & Installation

1.  **Install ROS 2 Humble and Dependencies:**
    ```bash
    # Install ROS 2, Gazebo, Nav2, SLAM, and other tools
    sudo apt update && sudo apt install -y ros-humble-desktop-full
    sudo apt install -y ros-humble-gazebo-ros-pkgs
    sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
    sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
    sudo apt install -y ros-humble-teleop-twist-keyboard
    ```

2.  **Create the Workspace:**
    The project follows a standard ROS 2 workspace structure. Use the commands in the [setup script](https://link-to-your-setup-script-if-you-create-one) or create it manually.

### Phase 1: Build & Control the Robot

The goal of this phase is to have a robot model that can be manually driven in the Gazebo simulation.

1.  **Develop the Environment & Robot:**
    * Edit `src/lunabot_pkg/worlds/lunar_habitat.world` to define the simulation environment.
    * Edit `src/lunabot_pkg/urdf/lunabot.urdf.xacro` to model the 4-wheel rover and add the necessary Gazebo plugins for sensors and control.
    * Edit `src/lunabot_pkg/launch/start_simulation.launch.py` to launch Gazebo, spawn the robot, and start necessary nodes like `robot_state_publisher`.

2.  **Build and Launch:**
    ```bash
    # Navigate to your workspace root
    cd ~/lunabot/ros2_ws/

    # Build the package
    colcon build --symlink-install

    # Source the workspace
    source install/setup.bash

    # Launch the simulation
    ros2 launch lunabot_pkg start_simulation.launch.py
    ```

3.  **Manual Control:**
    In a new terminal (after sourcing the workspace), run the teleop node to drive the robot with your keyboard.
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

### Phase 2: Autonomy (SLAM & Nav2)

This phase focuses on giving the robot a brain to map and navigate on its own.

1.  **Mapping with SLAM:**
    * With the simulation running, launch the SLAM toolbox.
    ```bash
    # Don't forget to source your workspace first!
    ros2 run slam_toolbox sync_slam_toolbox_node --ros-args -p use_sim_time:=true
    ```
    * Drive the robot around in `teleop` to build the map, then save it.
    ```bash
    # In a new terminal
    ros2 run nav2_map_server map_saver_cli -f ~/lunabot/ros2_ws/src/lunabot_pkg/config/my_map
    ```

2.  **Navigation with Nav2:**
    * Configure the `src/lunabot_pkg/config/nav2_params.yaml` file with the specifics of your robot.
    * Create a new launch file that starts the full Nav2 stack and loads your saved map.
    * Test by setting navigation goals in RViz.

### Phase 3: The Mission

Implement the specific patrol and monitoring logic.

1.  **Develop the Mission Node:**
    * Edit the mission script at `src/lunabot_pkg/scripts/patrol_and_monitor_node.py`.
    * The script will define patrol points, send goals to Nav2, and subscribe to sensor topics to check for anomalies.

### Phase 4: Polish & Presentation

The final phase is about refining the project for the final demonstration.

1.  **Refine the Simulation:** Improve the visual quality of the Gazebo world with textures and more complex models.
2.  **Create a Demo Video:** Record a clear video showcasing all the project's features, from mapping to the final mission execution.