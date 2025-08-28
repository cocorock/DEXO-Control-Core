# Bluetooth Smart Crutch Node

This node receives data from an ESP32-based smart crutch via Bluetooth RFCOMM connection and publishes ROS topics for integration with the exoskeleton control system.

## Prerequisites

1. **ESP32 Setup**: Flash the ESP32 with `MultisensorESP32_PT-CRUTCH_HMI-SYSTEM_V2.ino` ensuring `USE_COMPACT_FORMAT` is enabled.

2. **Bluetooth Pairing**: Pair the ESP32 device with your Linux system (device name: `CrutchHMI-BT-xxxx`).

3. **RFCOMM Connection**: Establish Bluetooth connection using the commands in `Docus/BT_Master_CMMs.txt`:
   ```bash
   sudo systemctl start bluetooth
   sudo hciconfig hci0 up
   sudo rfcomm connect 0 24:6F:28:D1:36:72 1
   sudo chmod 666 /dev/rfcomm0
   ```

## Usage

### Automated Launch (Recommended)
```bash
# One-command automated setup and launch
./src/exoskeleton_control/scripts/launch_bluetooth_crutch.sh

# Or using the automated launch file
roslaunch exoskeleton_control bluetooth_crutch_auto.launch

# Alternative: Setup only (without launching ROS)
./src/exoskeleton_control/scripts/launch_bluetooth_crutch.sh --setup-only

# Check connection status
./src/exoskeleton_control/scripts/launch_bluetooth_crutch.sh --status

# Cleanup connections
./src/exoskeleton_control/scripts/launch_bluetooth_crutch.sh --cleanup
```

### Manual Launch (Original Method)
```bash
# Manual Bluetooth setup
sudo systemctl start bluetooth
sudo hciconfig hci0 up
sudo rfcomm connect 0 24:6F:28:D1:36:72 1
# In another terminal:
sudo chmod 666 /dev/rfcomm0
roslaunch exoskeleton_control bluetooth_crutch.launch

# View published topics
rostopic echo /crutch/state
rostopic echo /crutch/sensor_data
```

### Integration with Exoskeleton System
The node can be integrated with existing launch files by including the Bluetooth crutch launch file or adding the node directly.

## Published Topics

### `/crutch/state` (CrutchState)
Publishes crutch state changes:
- **WALKING** ("W"): Analog input below threshold (user walking)
- **STANDING** ("S"): Analog input in middle range (user standing)
- **EMERGENCY_STOP** ("ES"): Analog input above threshold (emergency stop triggered)

### `/crutch/sensor_data` (CrutchSensorData)
Publishes continuous sensor data at ~20Hz:
- `force` (float32): Force measurement in grams
- `orientation` (geometry_msgs/Quaternion): Crutch orientation as quaternion (w,x,y,z)

## Data Format

The ESP32 sends data in compact CSV format:
```
timestamp,analog_command,force,qw,qx,qy,qz
```

Where:
- `timestamp`: ESP32 milliseconds timestamp
- `analog_command`: W/S/ES state
- `force`: Integer (actual force × 100)
- `qw,qx,qy,qz`: Quaternion as integers (actual × 1000)

## Configuration

Edit `config/bluetooth_crutch_config.yaml`:

```yaml
bluetooth_device: "/dev/rfcomm0"  # Bluetooth device path
baud_rate: 921600                 # Serial communication speed
reconnect_interval: 5.0           # Auto-reconnection delay (seconds)
publish_rate: 20                  # Target publish rate (Hz)
debug_output: true                # Enable verbose logging
```

## ESP32 Button Functions

When connected, the ESP32 supports these button commands:
- **B1 (Pin 4)**: Start motor calibration + normal operation
- **B2 (Pin 0)**: Begin force sensor calibration sequence
- **B3 (Pin 15)**: TARE during calibration / Print analog value during operation
- **B4 (Pin 2)**: Reset emergency stop (return to IDLE)

## Troubleshooting

### Connection Issues
```bash
# Check Bluetooth status
sudo systemctl status bluetooth

# Re-establish RFCOMM connection
sudo rfcomm release 0
sudo rfcomm connect 0 24:6F:28:D1:36:72 1

# Check device permissions
ls -l /dev/rfcomm0
sudo chmod 666 /dev/rfcomm0
```

### Node Debugging
- Set `debug_output: true` in config for verbose logging
- Check ESP32 serial output for debugging info
- Monitor ROS topics: `rostopic list | grep crutch`

### Data Validation
- Expected data rate: ~20Hz sensor data
- Force readings should be reasonable (typically 0-2000g)
- Quaternion should be normalized (w²+x²+y²+z² ≈ 1)

## Integration Example

To integrate with the 2-motor system, modify `core_nodes_2motors.launch`:

```xml
<!-- Include Bluetooth Crutch Node -->
<include file="$(find exoskeleton_control)/launch/bluetooth_crutch.launch"/>

<!-- Remap topics for integration -->
<remap from="/crutch/state" to="/exoskeleton/crutch_state"/>
<remap from="/crutch/sensor_data" to="/exoskeleton/crutch_sensors"/>
```

The emergency stop node can subscribe to `/crutch/state` to monitor for "ES" commands.