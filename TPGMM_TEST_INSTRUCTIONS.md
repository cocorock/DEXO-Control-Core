# TPGMM Trajectory Generator - Complete Testing Instructions

## Overview
This document provides step-by-step instructions for testing the TPGMM (Task-Parameterized Gaussian Mixture Models) trajectory generator node. The testing framework allows validation of the trajectory generator without requiring the full exoskeleton hardware setup.

## Prerequisites

### System Requirements
- ROS1 (tested with ROS Noetic)
- Python 3.x
- Required Python packages: `numpy`, `scipy`, `json`

### Installation Check
```bash
# Verify Python dependencies
python3 -c "import numpy, scipy, json; print('✓ All dependencies available')"

# Build workspace
cd /home/jemajuinta/ws/DEXO-Control-Core
catkin_make

# Source workspace
source devel/setup.bash
```

### File Structure Verification
Ensure these files exist:
```
src/exoskeleton_control/
├── scripts/
│   ├── tpgmm_trajectory_generator_node.py      # Main TPGMM node
│   ├── test_fsm_publisher.py                   # Automatic state publisher
│   ├── trajectory_monitor.py                   # Real-time monitoring
│   └── manual_tpgmm_test.py                    # Interactive testing
├── launch/
│   └── test_tpgmm_trajectory_generator.launch  # Test launch file
├── config/
│   └── tpgmm_trajectory_generator_config.yaml  # TPGMM configuration
├── data/4D/
│   └── gait_analysis_export_subject35.json     # Gait analysis data
└── pkls/
    └── gait_tpgmm_model_final.pkl              # TPGMM model (if available)
```

## Testing Procedures

### Test 1: Basic Automated Testing

**Purpose**: Verify TPGMM node loads correctly and generates trajectories automatically.

**Steps**:
1. **Launch the test environment**:
   ```bash
   roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch
   ```

2. **Expected Console Output**:
   ```
   [INFO] tpgmm: TPGMM Trajectory Generator Node initialized
   [INFO] tpgmm: Gait analysis data loaded from: .../gait_analysis_export_subject35.json
   [INFO] tpgmm: Gait analysis parameters loaded:
   [INFO] tpgmm: - Velocity multiplier: 0.25
   [INFO] tpgmm: - Interpolation points: 200
   [INFO] tpgmm: - Original duration: 1.084s
   [INFO] tpgmm: - Scaled duration: 4.335s
   [INFO] tpgmm: - Total interpolated points: 433
   [INFO] tpgmm: Trajectory timing configured for 4.335s cycles at 100Hz
   ```

3. **Verify State Transitions**:
   The test will automatically cycle through states every 5 seconds:
   ```
   [INFO] Published FSM state: INIT (index: 0)
   [INFO] Published FSM state: READY (index: 1)
   [INFO] Published FSM state: WALKING (index: 2)
   [INFO] tpgmm: State transition: READY -> WALKING
   [INFO] tpgmm: Started TPGMM trajectory for WALKING state
   [INFO] Published FSM state: STOPPING (index: 3)
   [INFO] Published FSM state: READY (index: 4)
   ```

4. **Monitor Real-time Statistics**:
   The trajectory monitor will display live statistics every 100ms:
   ```
   ================================================================================
   TPGMM TRAJECTORY MONITOR - REAL-TIME STATISTICS
   ================================================================================
   
   JOINT TRAJECTORIES (Total messages: 1250)
   --------------------------------------------------
   Hip Position  - Current:   0.123 rad (  7.0°)
                 - Range: [-0.175,  0.873] rad
   Knee Position - Current:  -0.654 rad (-37.5°)
                 - Range: [-1.047,  0.000] rad
   
   ANKLE TRAJECTORIES (Total messages: 1250)
   --------------------------------------------------
   Right Ankle   - Current: ( 0.045, -0.720) m
   Left Ankle    - Current: (-0.032, -0.685) m
   Time Phase    - Current:  0.573
   
   PUBLISHING RATES
   --------------------
   Joint Trajectories:  100.2 Hz
   Ankle Trajectories:   99.8 Hz
   ```

**Success Criteria**:
- ✅ Node initializes without errors
- ✅ Gait analysis data loads successfully (4.335s duration)
- ✅ State transitions trigger trajectory generation
- ✅ Publishing rates are ~100Hz
- ✅ Joint angles within expected ranges
- ✅ Time phase cycles from 0.0 to 1.0

### Test 2: Topic Monitoring

**Purpose**: Verify trajectory data is published correctly on all topics.

**Steps**:
1. **Keep Test 1 running**, then open new terminals for monitoring.

2. **Monitor Joint Trajectories**:
   ```bash
   # Check joint trajectory messages
   rostopic echo /test/joints_trajectory -n 5
   
   # Expected output structure:
   # header: {...}
   # Rhip_pos_ref: 0.123
   # Rknee_pos_ref: -0.654
   # Rhip_vel_ref: 0.045
   # Rknee_vel_ref: -0.123
   # Lhip_pos_ref: 0.123    # (mirrored from right)
   # Lknee_pos_ref: -0.654  # (mirrored from right)
   # Lhip_vel_ref: 0.045
   # Lknee_vel_ref: -0.123
   ```

3. **Monitor Dual Ankle Trajectories**:
   ```bash
   # Check dual ankle trajectory messages
   rostopic echo /test/dual_ankle_trajectory -n 5
   
   # Expected output structure:
   # header: {...}
   # right_ankle_pos_x: 0.045
   # right_ankle_pos_y: -0.720
   # right_ankle_vel_x: 0.123
   # right_ankle_vel_y: -0.045
   # left_ankle_pos_x: -0.032
   # left_ankle_pos_y: -0.685
   # left_ankle_vel_x: -0.089
   # left_ankle_vel_y: 0.067
   # time_phase: 0.573
   ```

4. **Check Publishing Rates**:
   ```bash
   # Verify joint trajectory rate (~100Hz)
   rostopic hz /test/joints_trajectory
   
   # Verify ankle trajectory rate (~100Hz)
   rostopic hz /test/dual_ankle_trajectory
   
   # Check FSM state rate (~0.2Hz - every 5 seconds)
   rostopic hz /test/fsm_state
   ```

**Success Criteria**:
- ✅ Joint trajectories publish at ~100Hz
- ✅ Ankle trajectories publish at ~100Hz
- ✅ All message fields contain reasonable values
- ✅ Time phase cycles smoothly from 0.0 to 1.0
- ✅ No NaN or infinite values in trajectories

### Test 3: Manual Interactive Testing

**Purpose**: Step-by-step validation of state transitions and emergency stop functionality.

**Steps**:
1. **Launch TPGMM node without automatic states**:
   ```bash
   roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch publish_test_states:=false
   ```

2. **Start manual test tool**:
   ```bash
   # In a new terminal
   rosrun exoskeleton_control manual_tpgmm_test.py
   ```

3. **Follow the interactive test sequence**:
   ```
   MANUAL TPGMM TRAJECTORY GENERATOR TEST
   ============================================================
   Available commands:
     State transitions:
       1: INIT
       2: CALIBRATION_PROCESS  
       3: READY
       4: WALKING
       5: STOPPING
       6: E_STOP
     Emergency stop:
       e: Trigger emergency stop
       r: Release emergency stop
     Other:
       h: Show this help
       q: Quit
   ============================================================
   ```

4. **Execute Test Sequence**:
   ```bash
   # Step 1: Initialize system
   Enter command: 1
   ✓ Published FSM state: INIT
   
   # Step 2: Go to ready state
   Enter command: 3  
   ✓ Published FSM state: READY
   
   # Step 3: Start walking (should begin trajectory generation)
   Enter command: 4
   ✓ Published FSM state: WALKING
   # Observe trajectory generation in main terminal
   
   # Step 4: Test emergency stop
   Enter command: e
   ✓ Triggered emergency stop
   # Trajectory generation should stop
   
   # Step 5: Release emergency stop
   Enter command: r
   ✓ Released emergency stop
   
   # Step 6: Return to ready
   Enter command: 3
   ✓ Published FSM state: READY
   
   # Step 7: Test stopping sequence
   Enter command: 4  # WALKING
   Enter command: 5  # STOPPING (should complete current cycle)
   
   # Step 8: Exit
   Enter command: q
   ```

5. **Monitor State Changes** (in main terminal):
   Look for these log messages:
   ```
   [INFO] tpgmm: State transition: INIT -> READY
   [INFO] tpgmm: State transition: READY -> WALKING
   [INFO] tpgmm: Started TPGMM trajectory for WALKING state
   [INFO] tpgmm: Received e_stop_trigger: True, state: MANUAL_TEST
   [INFO] tpgmm: State transition: WALKING -> STOPPING
   [INFO] tpgmm: TPGMM trajectory completed in STOPPING state
   [INFO] tpgmm: Sent cycle finished signal
   ```

**Success Criteria**:
- ✅ All state transitions work correctly
- ✅ WALKING state triggers trajectory generation
- ✅ Emergency stop immediately halts trajectories
- ✅ STOPPING state completes current cycle before stopping
- ✅ State transitions logged correctly

### Test 4: Configuration Testing

**Purpose**: Verify different configuration options work correctly.

**Steps**:
1. **Test with Different Frequencies**:
   ```bash
   # Test at 50Hz
   roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch control_frequency:=50
   
   # Verify rate: rostopic hz /test/joints_trajectory
   # Should show ~50Hz
   ```

2. **Test Without Gait Analysis Timing**:
   ```bash
   # Test with fallback duration (1.0s instead of 4.335s)
   roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch \
     gait_timing/use_gait_analysis_timing:=false
   
   # Look for log message:
   # [INFO] tpgmm: Gait analysis timing disabled - using fallback duration
   ```

3. **Test With Missing Files**:
   ```bash
   # Test with invalid gait analysis file
   roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch \
     gait_analysis_file:="/nonexistent/file.json"
   
   # Should gracefully fall back to default timing:
   # [WARN] tpgmm: Gait analysis file not found - using fallback duration: 1.0s
   ```

**Success Criteria**:
- ✅ Different frequencies work correctly
- ✅ Fallback timing works when gait analysis disabled
- ✅ Graceful handling of missing files
- ✅ No crashes or errors with invalid configurations

## Expected Results Summary

### Normal Operation Values
| Parameter | Expected Range | Notes |
|-----------|----------------|-------|
| Hip Position | -10° to 50° (-0.175 to 0.873 rad) | Joint limits enforced |
| Knee Position | -60° to 0° (-1.047 to 0 rad) | Joint limits enforced |
| Ankle Position X | -0.5 to 0.5 m | Relative to hip |
| Ankle Position Y | -0.9 to -0.2 m | Negative = below hip |
| Time Phase | 0.0 to 1.0 | Cycles continuously |
| Publishing Rate | ~100 Hz | Configurable |
| Cycle Duration | 4.335 seconds | From gait analysis |

### Performance Metrics
- **Trajectory Smoothness**: No sudden jumps or discontinuities
- **Timing Accuracy**: Cycle completes in exactly 4.335 seconds
- **Publishing Consistency**: Rate variation < 5%
- **Resource Usage**: CPU < 10%, Memory < 100MB

## Troubleshooting Guide

### Issue 1: Node Fails to Start
**Symptoms**: Node exits immediately or with import errors
**Solutions**:
```bash
# Check Python dependencies
python3 -c "import numpy, scipy, json"

# Verify file permissions
ls -la src/exoskeleton_control/scripts/tpgmm_trajectory_generator_node.py

# Check ROS environment
echo $ROS_PACKAGE_PATH
```

### Issue 2: No TPGMM Model Found
**Symptoms**: 
```
[ERROR] tpgmm: TPGMM model file not found in any of these locations
```
**Solutions**:
- The TPGMM model file may not be available in the repository
- Node will continue without the model but won't generate real trajectories
- This is expected if the trained model hasn't been provided

### Issue 3: No Trajectory Output
**Symptoms**: Topics publish but with zero values
**Possible Causes**:
1. **Wrong FSM State**: Trajectories only generate in WALKING/STOPPING states
2. **Emergency Stop Active**: Check emergency stop status
3. **Model Loading Failed**: Check model file availability

**Solutions**:
```bash
# Check current FSM state
rostopic echo /test/fsm_state -n 1

# Manually trigger WALKING state
rosrun exoskeleton_control manual_tpgmm_test.py
# Then press '4' for WALKING state

# Check for emergency stop
rostopic echo /test/e_stop_trigger -n 1
```

### Issue 4: Low Publishing Rate
**Symptoms**: `rostopic hz` shows rate < 90Hz
**Solutions**:
```bash
# Check system load
top

# Reduce trajectory monitor frequency
# Edit trajectory_monitor.py, change update_rate to 1.0

# Test with higher frequency
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch control_frequency:=50
```

### Issue 5: Gait Analysis File Not Found
**Symptoms**:
```
[WARN] tpgmm: Gait analysis file not found - using fallback duration: 1.0s
```
**Solutions**:
- This is expected behavior - node falls back to 1.0s cycle duration
- Verify file exists: `ls -la src/exoskeleton_control/data/4D/gait_analysis_export_subject35.json`
- Check file permissions: `chmod 644 src/exoskeleton_control/data/4D/gait_analysis_export_subject35.json`

## Advanced Testing

### Integration Testing
After successful standalone testing, test with other components:

```bash
# Test with 4-motor system (if available)
roslaunch exoskeleton_control core_nodes_4motors_tpgmm.launch

# Test with visualization
roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch enable_plots:=true
```

### Performance Testing
```bash
# Test for extended periods
timeout 300 roslaunch exoskeleton_control test_tpgmm_trajectory_generator.launch

# Monitor resource usage
htop
```

### Custom Configuration Testing
Create custom test configurations by modifying:
- `src/exoskeleton_control/config/tpgmm_trajectory_generator_config.yaml`
- Test different joint limits, frequencies, and timing parameters

## Cleanup

After testing, clean up processes:
```bash
# Kill all test nodes
rosnode kill -a

# Or restart roscore if needed
killall roscore rosmaster
roscore &
```

## Next Steps

After successful testing:
1. **Integration Testing**: Test with motor control nodes
2. **Hardware Testing**: Deploy on actual exoskeleton system  
3. **Performance Optimization**: Tune parameters for real-world usage
4. **Safety Validation**: Test emergency stop and limit behaviors

## Contact Information

For issues or questions regarding TPGMM testing:
- Check log outputs for detailed error messages
- Verify all prerequisites are met
- Test individual components in isolation
- Refer to ROS documentation for topic/node debugging

---
**Document Version**: 1.0  
**Last Updated**: Created with TPGMM implementation  
**Tested On**: ROS Noetic, Ubuntu 20.04