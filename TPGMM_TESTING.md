# TPGMM Trajectory Generator Testing Guide

This guide explains how to test the TPGMM trajectory generator node in isolation without requiring the full exoskeleton hardware setup.

## Test Files Created

### Launch Files
- **`test_tpgmm_trajectory_generator.launch`** - Main test launch file
- **`core_nodes_4motors_tpgmm.launch`** - Full 4-motor system launch file

### Test Scripts
- **`test_fsm_publisher.py`** - Automatic FSM state transitions for testing
- **`trajectory_monitor.py`** - Real-time trajectory statistics display
- **`manual_tpgmm_test.py`** - Interactive manual testing tool

## Basic Testing

### 1. Simple TPGMM Node Test
Test the TPGMM trajectory generator with automatic state transitions:

```bash
# Build and source workspace
catkin_make
source devel/setup.bash

# Launch TPGMM test with automatic state transitions
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch
```

This will:
- Load the TPGMM trajectory generator node
- Automatically cycle through FSM states (INIT → READY → WALKING → STOPPING → READY)
- Display real-time trajectory statistics
- Publish on test topics (`/test/*`)

### 2. Monitor Test Topics
In separate terminals, monitor the generated trajectories:

```bash
# Monitor joint trajectories
rostopic echo /test/joints_trajectory

# Monitor dual ankle trajectories  
rostopic echo /test/dual_ankle_trajectory

# Monitor FSM state transitions
rostopic echo /test/fsm_state

# Check publishing rates
rostopic hz /test/joints_trajectory
rostopic hz /test/dual_ankle_trajectory
```

### 3. Manual Interactive Testing
Run the manual test tool for step-by-step testing:

```bash
# Launch TPGMM node without automatic state transitions
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch publish_test_states:=false

# In another terminal, run manual test tool
rosrun exoskeleton_control manual_tpgmm_test.py
```

Manual test commands:
- `1-6`: Send FSM states (INIT, CALIBRATION_PROCESS, READY, WALKING, STOPPING, E_STOP)
- `e`: Trigger emergency stop
- `r`: Release emergency stop  
- `h`: Show help
- `q`: Quit

## Advanced Testing

### 4. Test with Visualization
Enable plotting and visualization:

```bash
# Launch with RQT plots
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch enable_plots:=true

# Launch with both plots and monitoring
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch enable_plots:=true enable_monitor:=true
```

### 5. Test Specific Scenarios

#### Test Gait Analysis Timing
```bash
# Test with gait analysis timing enabled (default)
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch

# Test with gait analysis timing disabled (fallback to 1.0s)
# Edit config file or use parameter override:
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch gait_timing/use_gait_analysis_timing:=false
```

#### Test Different Frequencies
```bash
# Test at 50Hz instead of 100Hz
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch control_frequency:=50

# Test at 200Hz
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch control_frequency:=200
```

## Expected Results

### Successful Test Indicators
1. **Node Initialization**:
   ```
   [INFO] tpgmm: TPGMM Trajectory Generator Node initialized
   [INFO] tpgmm: Loaded TPGMM model with X components
   [INFO] tpgmm: Trajectory timing configured for 4.335s cycles at 100Hz
   ```

2. **Gait Analysis Data Loading**:
   ```
   [INFO] tpgmm: Gait analysis parameters loaded:
   [INFO] tpgmm: - Velocity multiplier: 0.25
   [INFO] tpgmm: - Interpolation points: 200
   [INFO] tpgmm: - Original duration: 1.084s
   [INFO] tpgmm: - Scaled duration: 4.335s
   [INFO] tpgmm: - Total interpolated points: 433
   ```

3. **State Transitions**:
   ```
   [INFO] tpgmm: State transition: READY -> WALKING
   [INFO] tpgmm: Started TPGMM trajectory for WALKING state
   ```

4. **Trajectory Generation**:
   ```
   [INFO] tpgmm: TPGMM trajectory t=0.25: hip=15.2°, knee=-45.8°
   ```

### Expected Publishing Rates
- **Joint Trajectories**: ~100 Hz
- **Dual Ankle Trajectories**: ~100 Hz
- **FSM States**: As triggered (automatic: every 5s)

### Expected Value Ranges
- **Hip Position**: -10° to 50° (-0.175 to 0.873 rad)
- **Knee Position**: -60° to 0° (-1.047 to 0 rad)
- **Ankle Position X**: Approximately -0.5 to 0.5 m
- **Ankle Position Y**: Approximately -0.9 to -0.2 m
- **Time Phase**: 0.0 to 1.0 (cycling)

## Troubleshooting

### Common Issues

1. **TPGMM Model Not Found**:
   ```
   [ERROR] tpgmm: TPGMM model file not found
   ```
   - Ensure `gait_tpgmm_model_final.pkl` exists in `/pkls/` directory
   - Check file path in configuration

2. **Gait Analysis Data Not Found**:
   ```
   [WARN] tpgmm: Gait analysis file not found - using fallback duration
   ```
   - Ensure `gait_analysis_export_subject35.json` exists in `/data/4D/`
   - Node will use 1.0s fallback duration

3. **No Trajectory Output**:
   - Check FSM state: trajectories only publish in WALKING/STOPPING states
   - Verify emergency stop is not active
   - Use manual test tool to trigger WALKING state

4. **Import Errors**:
   ```
   [ERROR] No module named 'scipy'
   ```
   - Install required dependencies: `pip3 install scipy numpy`

### Debug Commands
```bash
# Check node status
rosnode info /tpgmm_trajectory_generator_node

# Check topic connections
rostopic info /test/joints_trajectory
rostopic info /test/dual_ankle_trajectory

# Check parameter loading
rosparam get /tpgmm_trajectory_generator_node

# Monitor logs
rosrun rqt_console rqt_console
```

## Integration Testing

After successful standalone testing, integrate with the full system:

```bash
# Test with 4-motor system
roslaunch exoskeleton_control core_nodes_4motors_tpgmm.launch

# Test with motor control node only (simulation)
roslaunch exoskeleton_control test_4motor_integration.launch
```

This testing framework provides comprehensive validation of the TPGMM trajectory generator functionality before deployment on the actual exoskeleton hardware.