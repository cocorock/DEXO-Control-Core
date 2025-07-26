# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System & Development Commands

This is a ROS1 Catkin workspace for a lower limb exoskeleton control system.

### Building the workspace
```bash
catkin_make
```

### Sourcing the workspace
```bash
source devel/setup.bash
```
Note: Must be done for every new terminal session.

### Running the system
```bash
# Launch all core nodes (motor control, emergency stop, trajectory generator, dummy crutches)
roslaunch exoskeleton_control core_nodes.launch

# Launch complete exoskeleton system (includes gait planner)
roslaunch exoskeleton_control exoskeleton_control.launch

# Launch individual nodes
roslaunch exoskeleton_control motor_control.launch
roslaunch exoskeleton_control emergency_stop.launch
roslaunch exoskeleton_control trajectory_generator.launch
roslaunch exoskeleton_control single_motor_test.launch
```

### Testing
```bash
# Test motor configuration loading
rosrun exoskeleton_control test_motor_config.py

# Test emergency stop functionality
rosrun exoskeleton_control test_emergency_stop.py

# Manual calibration trigger
rostopic pub /calibration_trigger std_msgs/Bool "data: true" --once

# Manual emergency stop
rostopic pub /stop_trigger std_msgs/Bool "data: true" --once
```

### Monitoring & Debugging
```bash
# Monitor system state
rostopic echo /e_stop_trigger
rostopic echo /MotorStatus
rostopic echo /ExoskeletonState

# Monitor feedforward torques (debugging)
rostopic echo /Torques

# List all topics
rostopic list
```

## Architecture Overview

This is a ROS1-based control system for a lower limb exoskeleton with the following key characteristics:

### Communication
- **CAN bus**: 1 Mbps for motor communication
- **Control frequency**: 100 Hz for motor control, 10-20 Hz for higher-level planning
- **State machine**: SMACH-based with states: Init → Calibrating → Ready → Stopped → Error

### Core Nodes

**Motor Control Node** (`motor_control_node.py`)
- Real-time motor control at 100 Hz via CAN bus
- Impedance control with internal feedforward torque calculation
- Controls 4 MIT Mini Cheetah motors (R_hip, R_knee, L_hip, L_knee)
- Publishes: ExoskeletonState, MotorStatus, Torques
- Subscribes: joints_trajectory, e_stop_trigger, calibration_trigger

**Emergency Stop Node** (`emergency_stop_node.py`)
- Safety monitoring and state machine management
- Monitors motor health and system state
- Publishes: e_stop_trigger with current system state
- Subscribes: MotorStatus, stop_trigger

**Trajectory Generator Node** (`trajectory_generator_node.py`)
- Converts gait parameters to joint trajectories using GMM/GMR
- Loads pre-computed trajectories from JSON data files
- Publishes: joints_trajectory
- Subscribes: gait_params, e_stop_trigger

**Gait Planner Node** (`gait_planner_node.py`)
- High-level gait planning and foot target calculation
- Currently deferred for initial testing
- Would publish: gait_params

**Dummy Crutches Node** (`dummy_crutches_node.py`)
- Manual testing interface via CLI
- Publishes: stop_trigger, calibration_trigger
- Interactive commands: 's' for stop, 'c' for calibration

### Key Message Types

**ExoskeletonState**: Current joint positions, velocities, and torques for both legs
**MotorStatus**: Individual motor status including calibration flags, temperature, and error flags
**JointsTrajectory**: Reference positions and velocities for all joints
**Torques**: Calculated feedforward and total command torques (debugging)
**EStopTrigger**: Emergency stop signal with current system state

### Configuration Files
- `motor_control_config.yaml`: Motor parameters, PID gains, calibration settings
- `emergency_stop_config.yaml`: Safety thresholds and state machine parameters
- `gait_planner_config.yaml`: Gait planning parameters
- `trajectory_generator_config.yaml`: Trajectory generation settings

### Data Files
Located in `data/` directory:
- `new_processed_gait_data#39_1.json`: Pre-computed gait trajectory data
- Used by trajectory generator for initial testing phase

### Network Configuration for Multi-Machine Setup
For distributed ROS deployment across multiple computers:
- Master computer: Set `ROS_MASTER_URI=http://[MASTER_IP]:11311` and `ROS_IP=[MASTER_IP]`
- Remote computers: Set `ROS_MASTER_URI=http://[MASTER_IP]:11311` and `ROS_IP=[REMOTE_IP]`
- Ensure network connectivity and hostname resolution between machines

### Important Implementation Details
- Motor control uses internal feedforward calculation to avoid service call overhead
- All timing-critical operations run at deterministic frequencies
- CAN communication timing budget: ~0.9ms for send/receive operations
- Safety margin of 2-3ms maintained in 100 Hz control loop
- System supports both single motor testing and full exoskeleton operation