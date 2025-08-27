# Simple Bot with Dynamic Obstacle Avoidance

This repository contains a **two-wheeled differential drive robot** with a caster wheel, defined in URDF, and configured with **Nav2** for dynamic obstacle avoidance.  
Simulation is carried out in **Gazebo**.

---

## Installation

Install the required ROS 2 packages (ROS Humble):

```bash
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

Create a workspace and clone the project:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vidhunvwarrier/simplebot_robot.git
```

Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Dynamic Obstacle Avoidance

The project uses **Nav2** for autonomous navigation with both **local** and **global** planners:

- **Local Map & Planner**  
  - Continuously updated with the current environment.  
  - Detects dynamic obstacles and re-plans around them.  
  - Uses the **Dynamic Window Approach (DWA) planner**, which is well-suited for handling moving obstacles.

- **Global Map & Planner**  
  - Built beforehand using **SLAM Toolbox**.  
  - Represents the static/original environment.  
  - Uses the **Dijkstra planner**, effective for indoor navigation.  

Thus, the **local planner ensures avoidance of dynamic obstacles** while following the global path.

---

## Dynamic Obstacle Simulation

- A moving **box (red cube)** is added in the world file.  
- A custom package (`obstacle_mover`) controls this box, making it move in a zig-zag pattern, simulating a dynamic obstacle in the environment.  

---

## Running the Project

### 1. Launch Gazebo with Robot
Start Gazebo with the robot URDF and predefined world:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch simplebot_description gazebo.launch.py
```

This also launches an **EKF** node that fuses odometry and IMU data for better localization.

---

### 2. Start Navigation
Run Nav2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch simplebot_description navigation.launch.py
```

- Open **RViz2**.  
- Set the initial pose using **2D Pose Estimate**.  
- Send navigation goals using **2D Goal Pose**.

---

### 3. Start the Moving Obstacle
Run the obstacle mover node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run obstacle_mover obstacle_mover_node
```

Now the red cube will start moving inside Gazebo.  
Try giving a navigation goal to the robot such that it encounters the moving obstacle. The robot should dynamically re-plan and avoid collisions.

---

## Demo


Video:

## Future Improvements
- Add support for different local planners (e.g., TEB).  
- Multi-robot navigation with obstacle interaction.  
- Real-world deployment on a physical robot platform.

---

## Author
**Vidhun V Warrier**  
[GitHub Profile](https://github.com/vidhunvwarrier)
