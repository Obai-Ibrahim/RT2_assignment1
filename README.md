# RT2 Assignment 1: ROS 2 Action Server Navigation

This repository contains a ROS 2 implementation of an action server designed to handle 2D robot navigation. The system is built using a component-based architecture to maximize performance and modularity in the ROS 2 ecosystem.

## 1. How to Run

Follow these steps to set up your workspace, clone the repository, and launch the navigation system.

### Step 1: Create a Workspace
Open your terminal and create a new ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Clone the Repository 
Clone the project and switch to the branch containing the completed assignment:

```bash
 mkdir -p ~/rt2_ws/src 
 cd ~/rt2_ws/src
 git clone -b complete_assignment  https://github.com/Obai-Ibrahim/ RT2_assignment1.git
 cd ~/rt2_ws
 colcon build 
 source install/setup.bash
```
### Step 3: Launch the Project
```bash
ros2 launch custom_interfaces as_components.py
```


## 2 Implementation Details

### 1. Client-Server Architecture
The project follows the **ROS 2 Action Design Pattern**:
*   **Action Server (C++):** A component-based node that receives navigation goals. It processes odometry data in real-time and computes velocity commands (`geometry_msgs/Twist`) to drive the robot.
*   **Action Client:** Can be any ROS 2 node or the command-line interface. The client sends a goal $(x, y, \theta)$ and receives continuous feedback on the remaining distance and heading error.

### 2. Custom Interface
The communication is governed by a custom action definition found in the `custom_interfaces` package.
**Action Definition (`Nav.action`):**
*   **Goal:** `float32 x`, `float32 y`, `float32 theta` (Target coordinates and orientation).
*   **Feedback:** `float32 err_pose`, `float32 err_th` (Current distance and angular error).
*   **Result:** `bool done` (Status of completion).

### 3. Component-Based Execution
Rather than running as a standard standalone executable, this node is implemented as a **ROS 2 Component**.
*   **Modularity:** The server is compiled into a shared library and registered using `RCLCPP_COMPONENTS_REGISTER_NODE`.
*   **Efficiency:** Running as a component allows the node to be loaded into a specialized container process at runtime. This enables zero-copy communication if other nodes are loaded into the same container, significantly reducing CPU overhead.

### 4. Concurrency Model
The server and client utilizes a **Reentrant Callback Group** and a **Multi-Threaded Executor** (if configured in the launch file).


---

## 3. Navigation Algorithm

The navigation logic is implemented using a **State-Based Proportional Controller**. The algorithm is divided into two distinct phases to handle 2D movement effectively.

### Phase 1: Translational Navigation (Approach)
In this phase, the robot focuses on reaching the target $(x, y)$ coordinate.
*   **Targeting:** The algorithm calculates the `angle_to_goal` using `atan2(delta_y, delta_x)`.
*   **Steering:** It computes a `steering_error` by comparing the robot's current yaw with the required heading. This error is normalized to $[-\pi, \pi]$ to ensure the robot turns the shortest way.
*   **Motion:** The robot moves forward with a linear velocity proportional to the distance error while simultaneously adjusting its angular velocity to stay pointed at the goal.
*   **Transition:** This phase remains active as long as the Euclidean distance error (`err_pose`) is greater than **0.5m**.

### Phase 2: Rotational Alignment (Final Heading)
Once the robot enters the 0.5m radius of the goal, it transitions to fine-tuning its orientation.
*   **Stop Motion:** Linear velocity is set to **0.0** to prevent the robot from "orbiting" the goal or drifting away.
*   **Alignment:** The algorithm calculates the difference between the requested `goal->theta` and the current `robot_theta_`.
*   **Rotation:** The robot rotates in place using a proportional gain.
*   **Termination:** The goal is marked as **Succeeded** only when the robot is within the tolerance threshold for both distance (< 0.05m) and orientation (< 0.05 rad).

### Mathematical Robustness
*   **Angle Normalization:** To prevent the robot from spinning $350^\circ$ when it only needs to turn $-10^\circ$, all angular errors are processed through a normalization loop:
    ```cpp
    while (error > M_PI)  error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;
    ```
*   **Velocity Clamping:** Both linear and angular velocities are clamped to safe maximums (e.g., 0.5 m/s and 0.5 rad/s) to prevent erratic behavior or hardware strain.