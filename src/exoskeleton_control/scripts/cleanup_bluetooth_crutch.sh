#!/bin/bash

# Bluetooth Dual Smart Crutch Cleanup Script
# This script cleans up existing Bluetooth RFCOMM connections for dual crutches
# Author: DEXO Control Core System
# Usage: ./cleanup_bluetooth_crutch.sh

echo "=========================================="
echo "Bluetooth Dual Smart Crutch Cleanup"
echo "=========================================="

# Define MAC addresses
ESP32_RIGHT_MAC="24:6F:28:D1:36:72"    # Right crutch (priority device)
ESP32_LEFT_MAC="24:6F:28:45:D3:76"     # Left crutch

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root for some operations
check_sudo() {
    if [ "$EUID" -eq 0 ]; then
        print_warning "Running as root. Some operations may require user permissions."
    fi
}

# Function to safely release RFCOMM binding
release_rfcomm() {
    local channel=$1
    local device_name=$2
    
    print_status "Releasing RFCOMM channel $channel ($device_name)..."
    
    if [ -e "/dev/rfcomm$channel" ]; then
        sudo rfcomm release $channel 2>/dev/null
        if [ $? -eq 0 ]; then
            print_status "Successfully released /dev/rfcomm$channel"
        else
            print_warning "Failed to release /dev/rfcomm$channel (may not be bound)"
        fi
    else
        print_status "/dev/rfcomm$channel does not exist"
    fi
}

# Function to kill processes using the devices
kill_device_processes() {
    local device=$1
    print_status "Checking for processes using $device..."
    
    local pids=$(lsof "$device" 2>/dev/null | awk 'NR>1 {print $2}' | sort -u)
    if [ -n "$pids" ]; then
        print_warning "Found processes using $device: $pids"
        echo -n "Kill these processes? (y/N): "
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            echo $pids | xargs -r sudo kill
            sleep 2
            # Force kill if still running
            echo $pids | xargs -r sudo kill -9 2>/dev/null
            print_status "Processes terminated"
        fi
    else
        print_status "No processes found using $device"
    fi
}

# Main cleanup sequence
main() {
    print_status "Starting Bluetooth dual crutch cleanup..."
    
    # Check for sudo capabilities
    check_sudo
    
    # Kill any ROS nodes that might be using the devices
    print_status "Stopping Bluetooth crutch nodes..."
    rosnode kill /bluetooth_crutch_node 2>/dev/null || print_status "No ROS bluetooth_crutch_node running"
    
    # Also kill direct Python processes
    print_status "Stopping direct Python bluetooth_crutch_node processes..."
    pkill -f "python3.*bluetooth_crutch_node.py" 2>/dev/null || print_status "No direct Python processes found"
    pkill -f "python.*bluetooth_crutch_node.py" 2>/dev/null || print_status "No direct Python processes found"
    
    # Check and kill processes using the devices
    if [ -e "/dev/rfcomm0" ]; then
        kill_device_processes "/dev/rfcomm0"
    fi
    
    if [ -e "/dev/rfcomm1" ]; then
        kill_device_processes "/dev/rfcomm1"
    fi
    
    # Release RFCOMM bindings
    release_rfcomm 0 "Right Crutch"
    release_rfcomm 1 "Left Crutch"
    
    # Additional cleanup - release any other RFCOMM channels that might exist
    print_status "Checking for additional RFCOMM channels..."
    for i in {2..9}; do
        if [ -e "/dev/rfcomm$i" ]; then
            release_rfcomm $i "Unknown Device"
        fi
    done
    
    # Reset Bluetooth adapter
    print_status "Resetting Bluetooth adapter..."
    sudo hciconfig hci0 down 2>/dev/null
    sleep 1
    sudo hciconfig hci0 up 2>/dev/null
    
    if [ $? -eq 0 ]; then
        print_status "Bluetooth adapter reset successfully"
    else
        print_error "Failed to reset Bluetooth adapter"
    fi
    
    # Optional: Remove paired devices (uncomment if needed)
    # print_status "Removing paired devices (optional)..."
    # bluetoothctl remove $ESP32_RIGHT_MAC 2>/dev/null || print_status "Right crutch not paired"
    # bluetoothctl remove $ESP32_LEFT_MAC 2>/dev/null || print_status "Left crutch not paired"
    
    # Restart Bluetooth service if needed
    print_status "Checking Bluetooth service status..."
    if ! systemctl is-active --quiet bluetooth; then
        print_warning "Bluetooth service is not active, attempting to restart..."
        sudo systemctl restart bluetooth
        sleep 2
    fi
    
    # Final status check
    print_status "Cleanup completed. Current RFCOMM status:"
    rfcomm show 2>/dev/null || print_status "No RFCOMM connections active"
    
    print_status "Current Bluetooth devices:"
    ls -la /dev/rfcomm* 2>/dev/null || print_status "No RFCOMM devices found"
    
    echo "=========================================="
    print_status "Bluetooth dual crutch cleanup completed!"
    print_status "You can now run:"
    print_status "  ./setup_bluetooth_crutch.sh (for setup)"
    print_status "  python3 bluetooth_crutch_node.py (to start node directly)"
    echo "=========================================="
}

# Handle script interruption
cleanup_on_exit() {
    print_warning "Script interrupted. Partial cleanup may have occurred."
    exit 1
}

# Set trap for clean exit
trap cleanup_on_exit INT TERM

# Run main function
main "$@"