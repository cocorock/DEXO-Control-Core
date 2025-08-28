# Bluetooth Master Connection Guide

## Connecting Ubuntu System as Master to Specific Device

To connect your Ubuntu system as a master to a specific device using its Bluetooth address:

### 1. Find the device address:
```bash
bluetoothctl
scan on
# Note the device address (XX:XX:XX:XX:XX:XX)
scan off
exit
```

### 2. Connect as master using rfcomm:
```bash
sudo rfcomm connect 0 XX:XX:XX:XX:XX:XX 1
```

### 3. Use the serial connection:
```bash
# In another terminal:
sudo minicom -b 9600 -o -D /dev/rfcomm0
```

### Alternative method using bluetoothctl:
```bash
bluetoothctl
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
exit
```

Replace `XX:XX:XX:XX:XX:XX` with the actual Bluetooth address of your target device. The target device must be in discoverable/connectable mode and support SPP.

## Troubleshooting "Failed to start discovery: org.bluez.Error.NotReady"

The Bluetooth adapter is not ready. Try these steps:

### 1. Check Bluetooth service status:
```bash
sudo systemctl status bluetooth
```

### 2. Start/restart Bluetooth service:
```bash
sudo systemctl start bluetooth
sudo systemctl restart bluetooth
```

### 3. Check if adapter is powered on:
```bash
bluetoothctl
power on
agent on
default-agent
scan on
```

### 4. If still not working, check adapter:
```bash
hciconfig
# Should show hci0 or similar
```

### 5. If no adapter found:
```bash
sudo hciconfig hci0 up
```

### 6. Reset Bluetooth if needed:
```bash
sudo systemctl stop bluetooth
sudo systemctl start bluetooth
```

Try `bluetoothctl` and `scan on` again after these steps.

## Configuring Line Endings for Serial Communication

If you experience formatting issues with line endings (text appearing on wrong lines), configure minicom:

### Configure minicom line endings:
```bash
sudo minicom -b 9600 -o -D /dev/rfcomm0

### Alternative: Use screen instead of minicom:
```bash
screen /dev/rfcomm0 9600
```

### Or use stty to configure the port:
```bash
sudo stty -F /dev/rfcomm0 9600 -echo -echoe -echok
sudo cat /dev/rfcomm0
```