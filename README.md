# ROS 2 MuJoCo Bridge for UR5e 🤖

A high-performance "Sim-to-Real" bridge connecting **ROS 2 (Humble)** with the **MuJoCo Physics Engine**. 
This project allows for direct control of a simulated UR5e robotic arm using standard ROS 2 topics, enabling safe algorithm testing before deployment on real hardware.

## 🚀 Features
* **Bi-Directional Communication:**
    * **Input:** Listens to `/joint_commands` to control the robot.
    * **Feedback:** Publishes `/joint_states` (Position & Velocity) from physics sensors.
* **Real-Time Simulation:** Runs MuJoCo physics steps synchronized with a ROS 2 timer.
* **Passive Viewer:** Integrated MuJoCo viewer for visual debugging without blocking the ROS loop.
* **Demo Controller:** Includes a `sine_wave_publisher` to demonstrate smooth kinematic control.

## 🛠️ Tech Stack
* **ROS 2 Distribution:** Humble Hawksbill
* **Simulation Engine:** MuJoCo (Python Native)
* **Robot Model:** Universal Robots UR5e
* **Language:** Python 3.10 (rclpy)

## 📦 Installation

1.  **Clone the Repository:**
    ```bash
    mkdir -p ~/ros2_mujoco_ws/src
    cd ~/ros2_mujoco_ws/src
    git clone [https://github.com/YOUR_USERNAME/ros2_mujoco_bridge.git](https://github.com/YOUR_USERNAME/ros2_mujoco_bridge.git)
    ```

2.  **Build the Workspace:**
    ```bash
    cd ~/ros2_mujoco_ws
    colcon build
    source install/setup.bash
    ```

## 🎮 Usage

### 1. Start the Bridge (The Robot)
This launches the MuJoCo simulation and starts the ROS node waiting for commands.
```bash
ros2 run mujoco_ur5_bridge start_bridge