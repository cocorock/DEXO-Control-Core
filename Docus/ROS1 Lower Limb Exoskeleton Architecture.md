ROS1 Lower Limb Exoskeleton Architecture
SYSTEM OVERVIEW
Platform: ROS1
Communication: CAN bus at 1 Mbps
Control Strategy: Impedance control with feedforward compensation
State Management: SMACH state machine (Init → Calibrating → Ready → Stopped → Error)
NODES
1. GAIT PLANNER NODE
Purpose: High-level gait planning and foot target calculation
Frequency: 10-20 Hz
Status: Deferred for initial testing

Inputs:
  - floor_ref: Floor plane reference (not implemented)
  - e_stop_trigger: Emergency stop signal

Outputs:
  - gait_params: Foot targets for both legs

Services Provided:
  - inverse_kinematics_closedchain: Closed-chain IK solver

Logic:
  - Calculate foot targets from floor plane
  - Right/left gait cycles with 50% phase offset
  - Update at end of each gait cycle

2. TRAJECTORY GENERATOR NODE
Purpose: Convert gait parameters to joint trajectories using GMM/GMR
Frequency: 15-20 Hz
Initial Mode: Load trajectories from JSON file

Inputs:
  - gait_params: Foot targets from gait planner
  - e_stop_trigger: Emergency stop signal

Outputs:
  - joints_trajectory: Position/velocity references for all joints

Services Used:
  - Forward_kinematics_leg: Convert joint angles to Cartesian
  - inverse_kinematics_leg: Convert Cartesian to joint angles

Logic:
  - Use GMM/GMR for ankle trajectory generation
  - Model each leg as 2-link planar arm
  - Convert between Cartesian and joint spaces

3. MOTOR CONTROL NODE
Purpose: Low-level motor control via CAN communication
Frequency: 100 Hz (10ms period breakdown below)

Timing Budget:
  - CAN send (4 frames): 0.444ms
  - CAN receive (4 frames): 0.444ms
  - Motor response buffer: 2ms
  - Feedforward torque calculation: 2-3ms (internal, no service overhead)
  - Impedance control calculation: 1-2ms
  - State publishing: 1ms
  - Safety margin: 2-3ms

Inputs:
  - joints_trajectory: Reference positions/velocities
  - e_stop_trigger: Emergency stop signal
  - calibration_trigger: Start calibration sequence

Outputs:
  - ExoskeletonState: Current joint states (pos, vel, torque)
  - MotorStatus: Individual motor status and health
  - Torques: Calculated feedforward torques (for monitoring/debugging)

Services Provided: None

Internal Functions:
  - calculate_feedforward_torques(): Internal dynamics calculation
  - dynamics_model_right_leg(): Right leg dynamics model
  - dynamics_model_left_leg(): Left leg dynamics model

Logic:
  - Read current motor states via CAN
  - Calculate feedforward torques INTERNALLY using dynamics models
  - Impedance control: τ = τ_ff + K_p*(q_ref - q) + K_d*(q̇_ref - q̇)
  - Send control commands to 4 motors synchronously
  - Publish current states and calculated torques for monitoring
  - Execute calibration routine when triggered
  - Block trajectory tracking until calibration complete

4. EMERGENCY STOP NODE
Purpose: System safety and state machine management
Frequency: Event-driven + periodic monitoring
State Machine: SMACH with states [Init, Calibrating, Ready, Stopped, Error]

Inputs:
  - MotorStatus: Motor health and error flags
  - stop_trigger: Manual stop command
  - Torso_pose: Torso orientation (not implemented)

Outputs:
  - e_stop_trigger: Emergency stop command + current state

Logic:
  - Monitor motor error_flags
  - Detect unsafe torso poses
  - Coordinate system state transitions
  - Immediate motor shutdown on emergency

5. DUMMY CRUTCHES NODE
Purpose: Manual testing interface
Status: Temporary for testing, future replacement planned

Inputs: None

Outputs:
  - stop_trigger: Manual stop command
  - calibration_trigger: Manual calibration start

Interface: CLI commands via rostopic pub

DATA STRUCTURES
TOPICS
e_stop_trigger:
  - trigger: bool
  - current_state: string
  - timestamp: rospy.Time

stop_trigger:
  - trigger: bool
  - timestamp: rospy.Time

calibration_trigger:
  - trigger: bool
  - timestamp: rospy.Time

gait_params:
  - right_target: {x: float, y: float}
  - left_target: {x: float, y: float}
  - timestamp: rospy.Time

joints_trajectory:
  - right_leg: {hip_pos: float, knee_pos: float, hip_vel: float, knee_vel: float}
  - left_leg: {hip_pos: float, knee_pos: float, hip_vel: float, knee_vel: float}
  - timestamp: rospy.Time

ExoskeletonState:
  - right_leg: {hip_pos: float, knee_pos: float, hip_vel: float, knee_vel: float, hip_torque: float, knee_torque: float}
  - left_leg: {hip_pos: float, knee_pos: float, hip_vel: float, knee_vel: float, hip_torque: float, knee_torque: float}
  - timestamp: rospy.Time

MotorStatus:
  - motors: [4 elements]
    - joint_name: string
    - motor_id: int
    - calibrated_flag: bool
    - position: float
    - velocity: float
    - torque: float
    - temperature: float
    - error_flags: int
  - timestamp: rospy.Time

Torques:
  - feedforward_torques: [4 floats]  # [R_hip, R_knee, L_hip, L_knee]
  - total_command_torques: [4 floats]  # [R_hip, R_knee, L_hip, L_knee] (optional for debugging)
  - timestamp: rospy.Time

SERVICES
inverse_kinematics_closedchain:
  Request: {leading_ankle_x: float, leading_ankle_y: float, hip_range: float, knee_range: float, torso_range: float}
  Response: {error_flag: bool, theta_right_hip: float, theta_left_hip: float, theta_right_knee: float, theta_left_knee: float, theta_torso: float}

inverse_kinematics_leg:
  Request: {ankle_x: float, ankle_y: float}
  Response: {theta1: float, theta2: float}

Forward_kinematics_leg:
  Request: {theta_hip: float, theta_knee: float}
  Response: {x: float, y: float}

MOTOR CONTROL NODE IMPLEMENTATION DETAILS
Control Loop Structure (100Hz)
def control_loop(self):
    rate = rospy.Rate(100)  # 100 Hz
    
    while not rospy.is_shutdown():
        if self.system_state == "Ready":
            # Step 1: Read motor states via CAN (0.444ms)
            current_states = self.read_motor_states()
            
            # Step 2: Calculate feedforward torques internally (2-3ms)
            ff_torques = self.calculate_feedforward_torques(
                positions=current_states.positions,
                velocities=current_states.velocities,
                accelerations=self.reference_accelerations
            )
            
            # Step 3: Calculate impedance control (1-2ms)
            control_torques = self.calculate_impedance_control(
                current_states, self.trajectory_references, ff_torques
            )
            
            # Step 4: Send commands to motors (0.444ms)
            self.send_motor_commands(control_torques)
            
            # Step 5: Publish states for monitoring (1ms)
            self.publish_states(current_states, ff_torques)
        
        rate.sleep()

def calculate_feedforward_torques(self, positions, velocities, accelerations):
    # Right leg dynamics (2-DOF)
    right_ff = self.dynamics_model_right_leg(
        positions[:2], velocities[:2], accelerations[:2]
    )
    
    # Left leg dynamics (2-DOF)  
    left_ff = self.dynamics_model_left_leg(
        positions[2:], velocities[2:], accelerations[2:]
    )
    
    return right_ff + left_ff

TESTING COMMANDS
# Manual calibration trigger
rostopic pub /calibration_trigger std_msgs/Bool "data: true" --once

# Manual emergency stop
rostopic pub /stop_trigger std_msgs/Bool "data: true" --once

# Monitor system state
rostopic echo /e_stop_trigger
rostopic echo /MotorStatus
rostopic echo /ExoskeletonState

# Monitor feedforward torques (for debugging)
rostopic echo /Torques

✅ PERFORMANCE BENEFITS
Eliminated service call overhead (~1-2ms saved per control cycle)
Deterministic timing for real-time control
Maintained observability through Torques topic publication
Better real-time performance with 2-3ms safety margin

This architecture now provides optimal real-time performance while maintaining full debugging capabilities!