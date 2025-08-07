### ROS System Description

This document outlines the ROS system for controlling a lower-limb exoskeleton, based on the `core_nodes_2motors.launch` file and the `dummy_crutches_node.py` script.

---

### 1. Depth Sensor Node (Not Implemented)

*   **Node Name:** `depth_sensor_node`
*   **Hardware:** Intel RealSense D435i
*   **Frequency:** ~30 Hz
*   **Function:** This node interfaces with the RealSense camera to perceive the environment. It processes the RGB-D data to detect the floor plane relative to the exoskeleton. It also publishes the orientation of the camera (IMU data), which is crucial for accurately interpreting the floor position.

*   **Outputs (Published Topics):**
    *   `/exoskeleton/floor_reference` (**sensor_msgs/Plane**): Publishes the detected floor plane equation.
    *   `/exoskeleton/camera_imu` (**sensor_msgs/Imu**): Publishes the orientation (roll, pitch, yaw) of the RealSense camera.

---

### 2. Gait Planner Node (Not Implemented)

*   **Node Name:** `gait_planner_node`
*   **Frequency:** 10-20 Hz
*   **Function:** This is a high-level planning node. It receives the floor reference from the `Depth Sensor Node` and the camera's orientation. Using the camera's pitch angle, it corrects the floor plane detection for any inclination. Based on this accurate floor data, it determines the optimal target position for the next footfall (for either the right or left leg) and sends these high-level targets to the `Trajectory Generator Node`.

*   **Inputs (Subscribed Topics):**
    *   `/exoskeleton/floor_reference` (**sensor_msgs/Plane**): The floor plane detected by the depth sensor.
    *   `/exoskeleton/camera_imu` (**sensor_msgs/Imu**): The orientation of the camera.
    *   `/exoskeleton/fsm_state` (**exoskeleton_control/FSMState**): To synchronize with the overall system state.

*   **Outputs (Published Topics):**
    *   `/exoskeleton/gait_params` (**exoskeleton_control/GaitParams**): Publishes the next footfall target (x, y position) for the Trajectory Generator.

---

### 3. Motor Control Node (2 motors)

*   **Node Name:** `motor_control_node_2motors`
*   **Script:** `motor_control_node_2motors.py`
*   **Frequency:** 100 Hz
*   **Function:** This node is the low-level controller for the two exoskeleton motors (right hip and right knee). It receives desired joint trajectories and translates them into commands for the motors. It also monitors the status of the motors and publishes their state. It operates as a state machine, transitioning between states like `IDLE`, `CALIBRATION`, `READY`, and `WALKING`.

*   **Inputs (Subscribed Topics):**
    *   `/exoskeleton/joints_trajectory` (**exoskeleton_control/JointsTrajectory**): Receives the desired trajectory for the joints from the Trajectory Generator node.
    *   `/exoskeleton/emergency_stop` (**exoskeleton_control/EStopTrigger**): Listens for an emergency stop signal from the Emergency Stop node.
    *   `/exoskeleton/fsm_state` (**exoskeleton_control/FSMState**): Receives state transitions from the Emergency Stop node to control its internal state machine.
    *   `/exoskeleton/calibration_trigger_fw` (**std_msgs/Trigger**): Receives a signal to start the calibration process.

*   **Outputs (Published Topics):**
    *   `/exoskeleton/state` (**exoskeleton_control/ExoskeletonState**): Publishes the current state of the exoskeleton, including joint positions, velocities, and torques.
    *   `/exoskeleton/motor_status` (**exoskeleton_control/MotorStatus**): Publishes detailed status information from the motors.
    *   `/exoskeleton/torques` (**exoskeleton_control/Torques**): Publishes the torques applied by the motors.
    *   `/exoskeleton/calibration_failed` (**std_msgs/Trigger**): Publishes a signal if the calibration process fails.
    *   `/exoskeleton/calibration_complete` (**std_msgs/Trigger**): Publishes a signal when the calibration process is successfully completed.

---

### 4. Emergency Stop Node

*   **Node Name:** `emergency_stop_node`
*   **Script:** `emergency_stop_node.py`
*   **Frequency:** 100 Hz
*   **Function:** This node acts as the master state machine for the entire system. It monitors the state of the exoskeleton and crutches to ensure safe operation. It is responsible for triggering emergency stops and managing the overall state transitions of the system (e.g., from `IDLE` to `WALKING`).

*   **Inputs (Subscribed Topics):**
    *   `/exoskeleton/state` (**exoskeleton_control/ExoskeletonState**): Monitors the current state of the exoskeleton.
    *   `/exoskeleton/motor_status` (**exoskeleton_control/MotorStatus**): Monitors the status of the motors.
    *   `/crutch_command` (**exoskeleton_control/CrutchCommand**): Receives commands from the crutches (or the dummy crutches node).
    *   `/exoskeleton/cycle_finished` (**std_msgs/Trigger**): Receives a signal when a gait cycle is complete.
    *   `/exoskeleton/calibration_failed` (**std_msgs/Trigger**): Listens for a calibration failure signal.
    *   `/exoskeleton/calibration_complete` (**std_msgs/Trigger**): Listens for a calibration completion signal.

*   **Outputs (Published Topics):**
    *   `/exoskeleton/emergency_stop` (**exoskeleton_control/EStopTrigger**): Publishes a trigger to immediately stop all motor activity.
    *   `/exoskeleton/fsm_state` (**exoskeleton_control/FSMState**): Publishes the current state of the finite state machine to coordinate other nodes.
    *   `/exoskeleton/calibration_trigger_fw` (**std_msgs/Trigger**): Triggers the calibration process in the motor control node.
    *   `/exoskeleton/walking_trigger` (**std_msgs/Trigger**): Signals the start of the walking sequence.
    *   `/exoskeleton/stopping_trigger` (**std_msgs/Trigger**): Signals the end of the walking sequence.
    *   `/exoskeleton/system_state` (**std_msgs/String**): Publishes a human-readable string of the current system state.

---

### 5. Trajectory Generator Node

*   **Node Name:** `trajectory_generator_node`
*   **Script:** `trajectory_generator_node.py`
*   **Frequency:** 200 Hz
*   **Function:** This node generates the joint trajectories for walking. It loads a pre-trained Gaussian Mixture Model (GMM) that represents a base gait pattern. Using Task-Parameterized GMM (TP-GMM), it adapts this base pattern in real-time based on high-level targets received from the `Gait Planner Node`. It starts and stops generating trajectories based on signals from the `Emergency Stop Node`.

*   **Inputs (Subscribed Topics):**
    *   `/exoskeleton/gait_params` (**exoskeleton_control/GaitParams**): Receives parameters to modify the gait, such as speed or step height.
    *   `/exoskeleton/emergency_stop` (**exoskeleton_control/EStopTrigger**): Listens for an emergency stop signal.
    *   `/exoskeleton/fsm_state` (**exoskeleton_control/FSMState**): Receives state transitions to know when to start or stop generating trajectories.

*   **Outputs (Published Topics):**
    *   `/exoskeleton/joints_trajectory` (**exoskeleton_control/JointsTrajectory**): Publishes the desired joint positions, velocities, and torques for the motor control node to follow.
    *   `/exoskeleton/cycle_finished` (**std_msgs/Trigger**): Publishes a signal at the end of each gait cycle.

---

### 6. Trajectory Plotter Node

*   **Node Name:** `trajectory_plotter_node`
*   **Script:** `trajectory_plotter_node.py`
*   **Frequency:** 10 Hz
*   **Function:** This is a debugging and visualization tool. It subscribes to the generated and actual joint trajectories and plots them in real-time using `matplotlib`, allowing for a visual comparison of the desired versus the actual motor movements.

*   **Inputs (Subscribed Topics):**
    *   `/exoskeleton/joints_trajectory` (**exoskeleton_control/JointsTrajectory**): The desired trajectory from the Trajectory Generator.
    *   `/exoskeleton/state` (**exoskeleton_control/ExoskeletonState**): The actual measured state from the Motor Control node.

*   **Outputs (Published Topics):**
    *   None. It generates plots on the screen.

---

### 7. Dummy Crutches Node

*   **Node Name:** `dummy_crutches_node`
*   **Script:** `dummy_crutches_node.py`
*   **Frequency:** Not fixed (runs in a loop waiting for user input).
*   **Function:** This node provides a command-line interface to manually send commands to the system, simulating the inputs that would normally come from physical crutches. It is used for testing and development to trigger state changes like `CALIBRATE`, `WALK`, and `STOP`.

*   **Inputs (Subscribed Topics):**
    *   None. It takes input from the user via the command line.

*   **Outputs (Published Topics):**
    *   `/crutch_command` (**exoskeleton_control/CrutchCommand**): Publishes commands to the Emergency Stop node to control the exoskeleton's state machine.
