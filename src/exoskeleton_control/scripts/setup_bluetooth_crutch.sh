#!/bin/bash

# Bluetooth Dual Smart Crutch Setup Script
# This script sets up Bluetooth RFCOMM connections for dual crutches
# Author: DEXO Control Core System
# Usage: ./setup_bluetooth_crutch.sh

echo "=========================================="
echo "Bluetooth Dual Smart Crutch Setup"
echo "=========================================="

# Define MAC addresses
ESP32_RIGHT_MAC="24:6F:28:D1:36:72"    # Right crutch (priority device)
ESP32_LEFT_MAC="24:6F:28:45:D3:76"     # Left crutch

# Configuration
RFCOMM_CHANNEL=1
TIMEOUT=30
MAX_RETRIES=3

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Function to check if Bluetooth is available
check_bluetooth() {
    print_step "Checking Bluetooth availability..."
    
    if ! command -v bluetoothctl &> /dev/null; then
        print_error "bluetoothctl not found. Please install bluez."
        return 1
    fi
    
    if ! hciconfig hci0 &> /dev/null; then
        print_error "Bluetooth adapter hci0 not found."
        return 1
    fi
    
    # Ensure Bluetooth service is running
    if ! systemctl is-active --quiet bluetooth; then
        print_warning "Bluetooth service not active, starting..."
        sudo systemctl start bluetooth
        sleep 2
    fi
    
    # Power on the adapter
    sudo hciconfig hci0 up
    print_status "Bluetooth adapter ready"
    return 0
}

# Function to scan for device with timeout
scan_for_device() {
    local mac_address=$1
    local device_name=$2
    local timeout=$3
    
    print_step "Scanning for $device_name ($mac_address)..."
    
    # Start scanning in background
    bluetoothctl scan on &
    local scan_pid=$!
    
    # Wait for device to be discovered
    local count=0
    while [ $count -lt $timeout ]; do
        if bluetoothctl devices | grep -q "$mac_address"; then
            print_status "$device_name found!"
            kill $scan_pid 2>/dev/null
            bluetoothctl scan off
            return 0
        fi
        sleep 1
        count=$((count + 1))
        printf "."
    done
    
    echo ""
    kill $scan_pid 2>/dev/null
    bluetoothctl scan off
    print_warning "$device_name not found after ${timeout}s"
    return 1
}

# Function to pair and trust device
pair_device() {
    local mac_address=$1
    local device_name=$2
    
    print_step "Pairing with $device_name ($mac_address)..."
    
    # Check if already paired
    if bluetoothctl info "$mac_address" | grep -q "Paired: yes"; then
        print_status "$device_name already paired"
    else
        # Attempt to pair
        if echo -e "pair $mac_address\nyes\n" | timeout 15 bluetoothctl; then
            print_status "$device_name paired successfully"
        else
            print_warning "Pairing with $device_name may have failed, continuing..."
        fi
    fi
    
    # Trust the device
    print_status "Trusting $device_name..."
    echo "trust $mac_address" | bluetoothctl
    
    return 0
}

# Function to bind RFCOMM device with retries
bind_rfcomm() {
    local rfcomm_number=$1
    local mac_address=$2
    local device_name=$3
    local max_attempts=$4
    
    print_step "Binding $device_name to /dev/rfcomm$rfcomm_number..."
    
    local attempt=1
    while [ $attempt -le $max_attempts ]; do
        print_status "Attempt $attempt/$max_attempts for $device_name..."
        
        # Try to bind
        if sudo rfcomm bind $rfcomm_number "$mac_address" $RFCOMM_CHANNEL; then
            print_status "$device_name bound to /dev/rfcomm$rfcomm_number successfully"
            
            # Set permissions
            sleep 1
            sudo chmod 666 "/dev/rfcomm$rfcomm_number"
            
            # Verify the device exists and is accessible
            if [ -c "/dev/rfcomm$rfcomm_number" ]; then
                print_status "/dev/rfcomm$rfcomm_number is ready"
                return 0
            else
                print_warning "/dev/rfcomm$rfcomm_number not accessible"
            fi
        else
            print_warning "Failed to bind $device_name (attempt $attempt)"
        fi
        
        attempt=$((attempt + 1))
        if [ $attempt -le $max_attempts ]; then
            print_status "Waiting 3 seconds before retry..."
            sleep 3
        fi
    done
    
    print_error "Failed to bind $device_name after $max_attempts attempts"
    return 1
}

# Function to test connection
test_connection() {
    local rfcomm_device=$1
    local device_name=$2
    
    print_step "Testing connection to $device_name..."
    
    if [ ! -c "$rfcomm_device" ]; then
        print_error "$rfcomm_device does not exist"
        return 1
    fi
    
    # Test if we can open the device (timeout after 5 seconds)
    if timeout 5 bash -c "exec 3<>$rfcomm_device && exec 3<&-"; then
        print_status "$device_name connection test passed"
        return 0
    else
        print_warning "$device_name connection test failed"
        return 1
    fi
}

# Function to setup single crutch
setup_crutch() {
    local mac_address=$1
    local rfcomm_number=$2
    local device_name=$3
    local is_critical=$4
    
    print_step "Setting up $device_name..."
    echo "MAC: $mac_address -> /dev/rfcomm$rfcomm_number"
    
    # Scan for device
    if ! scan_for_device "$mac_address" "$device_name" $TIMEOUT; then
        if [ "$is_critical" = "true" ]; then
            print_error "$device_name is critical and was not found!"
            return 1
        else
            print_warning "$device_name not found, continuing without it..."
            return 2  # Non-critical failure
        fi
    fi
    
    # Pair and trust
    if ! pair_device "$mac_address" "$device_name"; then
        print_error "Failed to pair $device_name"
        if [ "$is_critical" = "true" ]; then
            return 1
        else
            return 2
        fi
    fi
    
    # Bind RFCOMM
    if ! bind_rfcomm $rfcomm_number "$mac_address" "$device_name" $MAX_RETRIES; then
        print_error "Failed to bind $device_name"
        if [ "$is_critical" = "true" ]; then
            return 1
        else
            return 2
        fi
    fi
    
    # Test connection
    if ! test_connection "/dev/rfcomm$rfcomm_number" "$device_name"; then
        print_warning "$device_name binding succeeded but connection test failed"
        if [ "$is_critical" = "true" ]; then
            return 1
        else
            return 2
        fi
    fi
    
    print_status "$device_name setup completed successfully!"
    return 0
}

# Function to run cleanup first
run_cleanup() {
    print_step "Running cleanup script first..."
    local cleanup_script="$(dirname "$0")/cleanup_bluetooth_crutch.sh"
    
    if [ -f "$cleanup_script" ]; then
        if [ -x "$cleanup_script" ]; then
            print_status "Executing cleanup script..."
            "$cleanup_script"
        else
            print_warning "Cleanup script not executable, making it executable..."
            chmod +x "$cleanup_script"
            "$cleanup_script"
        fi
    else
        print_warning "Cleanup script not found at $cleanup_script"
        print_status "Performing basic cleanup..."
        sudo rfcomm release 0 2>/dev/null || true
        sudo rfcomm release 1 2>/dev/null || true
    fi
}

# Main setup function
main() {
    print_status "Starting Bluetooth dual crutch setup..."
    print_status "Right Crutch (Priority): $ESP32_RIGHT_MAC -> /dev/rfcomm0"
    print_status "Left Crutch: $ESP32_LEFT_MAC -> /dev/rfcomm1"
    
    # Run cleanup first
    run_cleanup
    echo ""
    
    # Check Bluetooth availability
    if ! check_bluetooth; then
        print_error "Bluetooth setup failed. Please check your Bluetooth configuration."
        exit 1
    fi
    
    # Setup variables to track success
    local right_success=false
    local left_success=false
    
    # Setup right crutch (critical - must succeed)
    print_step "=== Setting up RIGHT CRUTCH (Priority Device) ==="
    if setup_crutch "$ESP32_RIGHT_MAC" 0 "Right Crutch" true; then
        right_success=true
    else
        print_error "Right crutch setup failed! This is a critical failure."
        exit 1
    fi
    
    echo ""
    
    # Setup left crutch (non-critical - system can work without it)
    print_step "=== Setting up LEFT CRUTCH ==="
    local left_result
    setup_crutch "$ESP32_LEFT_MAC" 1 "Left Crutch" false
    left_result=$?
    
    if [ $left_result -eq 0 ]; then
        left_success=true
    elif [ $left_result -eq 2 ]; then
        print_warning "Left crutch setup failed, but system can continue with right crutch only"
    else
        print_error "Left crutch setup encountered critical error"
    fi
    
    echo ""
    
    # Summary
    print_step "=== SETUP SUMMARY ==="
    if [ "$right_success" = true ]; then
        print_status "✓ Right Crutch: Successfully connected to /dev/rfcomm0"
    else
        print_error "✗ Right Crutch: Failed"
    fi
    
    if [ "$left_success" = true ]; then
        print_status "✓ Left Crutch: Successfully connected to /dev/rfcomm1"
    else
        print_warning "✗ Left Crutch: Not connected (system will operate in single-crutch mode)"
    fi
    
    # Show current RFCOMM status
    echo ""
    print_step "Current RFCOMM Status:"
    rfcomm show
    
    echo ""
    print_step "Available devices:"
    ls -la /dev/rfcomm* 2>/dev/null || print_status "No RFCOMM devices found"
    
    echo ""
    echo "=========================================="
    if [ "$right_success" = true ]; then
        print_status "Setup completed! You can now start the system with:"
        echo ""
        echo "Option 1 - Direct Python node (recommended):"
        echo "cd $(dirname "$0") && python3 bluetooth_crutch_node.py"
        echo ""
        echo "Option 2 - Using rosrun:"
        echo "rosrun exoskeleton_control bluetooth_crutch_node.py"
        echo ""
        echo "Option 3 - Auto-start after this script:"
        echo -n "Start Bluetooth crutch node now? (y/N): "
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            print_status "Starting Bluetooth crutch node..."
            cd "$(dirname "$0")"
            export ROS_NAMESPACE="/bluetooth_crutch_node"
            rosparam set bluetooth_right_device "/dev/rfcomm0"
            rosparam set bluetooth_left_device "/dev/rfcomm1"
            rosparam set baud_rate 115200
            rosparam set debug_output true
            python3 bluetooth_crutch_node.py
        fi
    else
        print_error "Setup failed! Please check the errors above."
        exit 1
    fi
    echo "=========================================="
}

# Handle script interruption
cleanup_on_exit() {
    print_warning "Setup interrupted. You may need to run cleanup script."
    exit 1
}

# Set trap for clean exit
trap cleanup_on_exit INT TERM

# Check if running with proper permissions
if [ "$EUID" -eq 0 ]; then
    print_warning "Running as root. This may cause permission issues."
    print_warning "Consider running as a regular user with sudo when needed."
fi

# Run main function
main "$@"