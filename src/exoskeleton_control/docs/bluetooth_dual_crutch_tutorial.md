# Bluetooth Dual Smart Crutch Setup Tutorial

## Overview

This tutorial provides comprehensive instructions for setting up and managing Bluetooth connections for dual smart crutches in the DEXO Control Core system. The system supports two ESP32-based smart crutches that communicate sensor data via Bluetooth.

## Device Configuration

### Crutch MAC Addresses
- **Right Crutch (Priority Device)**: `24:6F:28:D1:36:72` → `/dev/rfcomm0`
- **Left Crutch**: `24:6F:28:45:D3:76` → `/dev/rfcomm1`

### Serial Configuration
- **Baud Rate**: 115200
- **Communication Protocol**: RFCOMM (Bluetooth Serial)
- **Data Rate**: 20 Hz

## Quick Start

### Automated Setup
```bash
# Clean up existing connections
./cleanup_bluetooth_crutch.sh

# Set up dual crutch connections
./setup_bluetooth_crutch.sh
```

### Start the System
```bash
# Option 1: Direct Python execution (recommended)
cd src/exoskeleton_control/scripts && python3 bluetooth_crutch_node.py

# Option 2: Using rosrun
rosrun exoskeleton_control bluetooth_crutch_node.py
```

## Manual Setup Instructions

### 1. Cleanup Existing Connections

**Remove existing RFCOMM bindings:**
```bash
sudo rfcomm release 0
sudo rfcomm release 1
```

**Reset Bluetooth adapter:**
```bash
sudo hciconfig hci0 down
sudo hciconfig hci0 up
```

**Remove paired devices (if needed):**
```bash
bluetoothctl remove 24:6F:28:D1:36:72
bluetoothctl remove 24:6F:28:45:D3:76
```

### 2. Pair and Trust Devices

**Start bluetoothctl:**
```bash
bluetoothctl
```

**In bluetoothctl console:**
```bash
# Make adapter discoverable
discoverable on
pairable on

# Scan for devices
scan on

# Pair right crutch (priority device)
pair 24:6F:28:D1:36:72
trust 24:6F:28:D1:36:72

# Pair left crutch
pair 24:6F:28:45:D3:76  
trust 24:6F:28:45:D3:76

# Exit bluetoothctl
exit
```

### 3. Create RFCOMM Connections

**Bind devices to RFCOMM channels:**
```bash
# Right crutch to rfcomm0 (priority device)
sudo rfcomm bind 0 24:6F:28:D1:36:72 1

# Left crutch to rfcomm1
sudo rfcomm bind 1 24:6F:28:45:D3:76 1
```

**Verify connections:**
```bash
rfcomm show
ls -la /dev/rfcomm*
```

### 4. Set Permissions

**Make devices accessible:**
```bash
sudo chmod 666 /dev/rfcomm0
sudo chmod 666 /dev/rfcomm1
```

## Troubleshooting

### Connection Issues

**Check Bluetooth status:**
```bash
hciconfig
systemctl status bluetooth
```

**Check device availability:**
```bash
bluetoothctl devices
bluetoothctl info 24:6F:28:D1:36:72
bluetoothctl info 24:6F:28:45:D3:76
```

**Test serial communication:**
```bash
# Test right crutch
cat /dev/rfcomm0

# Test left crutch  
cat /dev/rfcomm1
```

### Common Problems

1. **Device not found**: Ensure ESP32 devices are powered on and in pairing mode
2. **Permission denied**: Run `sudo chmod 666 /dev/rfcomm*`
3. **Connection timeout**: Check MAC addresses and ensure devices are in range
4. **Port busy**: Run cleanup script to release existing connections

### Fallback Mode

The system automatically falls back to single-crutch mode if the left crutch fails to connect after 10 attempts. The right crutch (priority device) will continue operating independently.

## System Architecture

### Device Priorities
1. **Right Crutch** (`/dev/rfcomm0`): Primary device, always required
2. **Left Crutch** (`/dev/rfcomm1`): Secondary device, optional for system operation

### Data Flow
```
ESP32 Crutches → Bluetooth → RFCOMM → ROS Node → System
```

### Topics Published
- `/crutch/state`: Combined crutch state information
- `/crutch/sensor_data`: Raw sensor data from both crutches

## Configuration Files

### Main Config
- `config/bluetooth_crutch_config.yaml`: System configuration

### Scripts
- `scripts/setup_bluetooth_crutch.sh`: Automated setup script
- `scripts/cleanup_bluetooth_crutch.sh`: Cleanup script  
- `scripts/bluetooth_crutch_node.py`: Main ROS node for dual crutch communication

## Safety Features

- **Emergency Stop**: System responds to emergency signals from either crutch
- **Connection Monitoring**: Continuous monitoring of both connections
- **Automatic Fallback**: Graceful degradation to single-crutch mode
- **Reconnection Logic**: Automatic reconnection attempts with configurable delays

## Performance Optimization

- **High Baud Rate**: 115200 bps for low latency
- **Efficient Data Format**: Optimized sensor data packaging
- **Connection Pooling**: Maintains persistent connections
- **Error Recovery**: Robust error handling and recovery mechanisms

## Manual Commands Reference

### Complete Manual Setup Sequence

**Step 1: Cleanup existing connections**
```bash
sudo rfcomm release 0
sudo rfcomm release 1
sudo hciconfig hci0 down
sudo hciconfig hci0 up
```

**Step 2: Start Bluetooth service**
```bash
sudo systemctl start bluetooth
sudo hciconfig hci0 up
```

**Step 3: Pair devices using bluetoothctl**
```bash
bluetoothctl
# In bluetoothctl:
scan on
# Wait for devices to appear, then:
pair 24:6F:28:D1:36:72
trust 24:6F:28:D1:36:72
pair 24:6F:28:45:D3:76
trust 24:6F:28:45:D3:76
exit
```

**Step 4: Create simultaneous RFCOMM connections**
```bash
# Bind both devices simultaneously
sudo rfcomm bind 0 24:6F:28:D1:36:72 1 &
sudo rfcomm bind 1 24:6F:28:45:D3:76 1 &
wait

# Set permissions
sudo chmod 666 /dev/rfcomm0 /dev/rfcomm1

# Verify connections
rfcomm show
ls -la /dev/rfcomm*
```

### Quick Commands for Daily Use

**Check connection status:**
```bash
rfcomm show
ls -la /dev/rfcomm*
bluetoothctl devices
```

**Test connections:**
```bash
# Test right crutch
timeout 5 cat /dev/rfcomm0 &

# Test left crutch
timeout 5 cat /dev/rfcomm1 &
```

**Emergency reset:**
```bash
sudo rfcomm release all
sudo systemctl restart bluetooth
sudo hciconfig hci0 up
```

## Script Usage

### Automated Setup (Recommended)
```bash
# Navigate to scripts directory
cd /path/to/src/exoskeleton_control/scripts/

# Run cleanup and setup
./cleanup_bluetooth_crutch.sh && ./setup_bluetooth_crutch.sh
```

### Individual Script Usage
```bash
# Cleanup only
./cleanup_bluetooth_crutch.sh

# Setup only (after cleanup)
./setup_bluetooth_crutch.sh
```

## Starting the System

### Direct Python Execution (Recommended)
```bash
# Navigate to scripts directory
cd src/exoskeleton_control/scripts/

# Set ROS parameters (if needed)
rosparam set /bluetooth_crutch_node/bluetooth_right_device "/dev/rfcomm0"
rosparam set /bluetooth_crutch_node/bluetooth_left_device "/dev/rfcomm1"
rosparam set /bluetooth_crutch_node/baud_rate 115200
rosparam set /bluetooth_crutch_node/debug_output true

# Start the node
python3 bluetooth_crutch_node.py
```

### Using rosrun
```bash
rosrun exoskeleton_control bluetooth_crutch_node.py
```

### Complete Workflow
```bash
# Complete setup and start sequence
cd src/exoskeleton_control/scripts/
./cleanup_bluetooth_crutch.sh
./setup_bluetooth_crutch.sh
# Answer 'y' when prompted to auto-start the node
```

## System Monitoring

### Check ROS Topics
```bash
# List crutch-related topics
rostopic list | grep crutch

# Watch crutch data
rostopic echo /crutch/sensor_data

# Check system state
rostopic echo /crutch/state
```

### Check ROS Nodes
```bash
# List running nodes
rosnode list | grep crutch

# Check node info
rosnode info /bluetooth_crutch_node
```

### Monitor Connection Status
```bash
# Check RFCOMM connections
rfcomm show

# Check device files
ls -la /dev/rfcomm*

# Monitor system logs
rostopic echo /rosout | grep bluetooth_crutch_node
```