#!/bin/bash

# Automated Bluetooth Smart Crutch Setup Script
# This script handles the complete Bluetooth connection setup for the smart crutch

set -e  # Exit on any error

# Configuration
ESP32_MAC_ADDRESS="24:6F:28:D1:36:72"
RFCOMM_CHANNEL="1"
RFCOMM_DEVICE="/dev/rfcomm0"
RFCOMM_ID="0"
SCRIPT_NAME="$(basename "$0")"
LOG_PREFIX="[BT-CRUTCH-SETUP]"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}${LOG_PREFIX} INFO: $1${NC}"
}

log_success() {
    echo -e "${GREEN}${LOG_PREFIX} SUCCESS: $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}${LOG_PREFIX} WARNING: $1${NC}"
}

log_error() {
    echo -e "${RED}${LOG_PREFIX} ERROR: $1${NC}"
}

# Function to check if running as root for sudo commands
check_sudo() {
    if [[ $EUID -eq 0 ]]; then
        SUDO_CMD=""
    else
        SUDO_CMD="sudo"
        log_info "Will use sudo for privileged operations"
    fi
}

# Function to check if Bluetooth service is running
check_bluetooth_service() {
    log_info "Checking Bluetooth service status..."
    
    if systemctl is-active --quiet bluetooth; then
        log_success "Bluetooth service is running"
        return 0
    else
        log_warning "Bluetooth service is not running, starting it..."
        $SUDO_CMD systemctl start bluetooth
        sleep 2
        
        if systemctl is-active --quiet bluetooth; then
            log_success "Bluetooth service started successfully"
            return 0
        else
            log_error "Failed to start Bluetooth service"
            return 1
        fi
    fi
}

# Function to bring up Bluetooth interface
setup_bluetooth_interface() {
    log_info "Setting up Bluetooth interface..."
    
    # Check if hci0 exists
    if ! hciconfig hci0 >/dev/null 2>&1; then
        log_error "Bluetooth interface hci0 not found"
        return 1
    fi
    
    # Bring up the interface
    $SUDO_CMD hciconfig hci0 up
    sleep 1
    
    # Check if interface is up
    if hciconfig hci0 | grep -q "UP RUNNING"; then
        log_success "Bluetooth interface hci0 is up and running"
        return 0
    else
        log_error "Failed to bring up Bluetooth interface hci0"
        return 1
    fi
}

# Function to check if RFCOMM connection already exists
check_existing_rfcomm() {
    if [ -e "$RFCOMM_DEVICE" ]; then
        log_info "RFCOMM device $RFCOMM_DEVICE already exists"
        
        # Check if it's actually connected and working
        if [ -r "$RFCOMM_DEVICE" ] && [ -w "$RFCOMM_DEVICE" ]; then
            log_success "Existing RFCOMM connection appears to be working"
            return 0
        else
            log_warning "RFCOMM device exists but may not be functional, cleaning up..."
            cleanup_rfcomm
            return 1
        fi
    fi
    return 1
}

# Function to cleanup existing RFCOMM connections
cleanup_rfcomm() {
    log_info "Cleaning up existing RFCOMM connections..."
    
    # Release any existing RFCOMM connections
    if $SUDO_CMD rfcomm show | grep -q "^rfcomm${RFCOMM_ID}"; then
        log_info "Releasing existing RFCOMM connection..."
        $SUDO_CMD rfcomm release $RFCOMM_ID || true
        sleep 1
    fi
    
    # Remove device file if it exists
    if [ -e "$RFCOMM_DEVICE" ]; then
        log_info "Removing existing RFCOMM device file..."
        $SUDO_CMD rm -f "$RFCOMM_DEVICE" || true
    fi
}

# Function to establish RFCOMM connection
establish_rfcomm_connection() {
    log_info "Establishing RFCOMM connection to $ESP32_MAC_ADDRESS..."
    
    # First check if ESP32 is discoverable/paired
    if ! hcitool scan | grep -q "$ESP32_MAC_ADDRESS"; then
        log_warning "ESP32 device $ESP32_MAC_ADDRESS not found in scan"
        log_info "Make sure the ESP32 is powered on and Bluetooth is enabled"
    fi
    
    # Start RFCOMM connection in background
    log_info "Connecting to ESP32 via RFCOMM..."
    nohup $SUDO_CMD rfcomm connect $RFCOMM_ID $ESP32_MAC_ADDRESS $RFCOMM_CHANNEL >/dev/null 2>&1 &
    RFCOMM_PID=$!
    
    # Wait for connection to establish
    log_info "Waiting for RFCOMM connection to establish..."
    for i in {1..10}; do
        if [ -e "$RFCOMM_DEVICE" ]; then
            log_success "RFCOMM connection established"
            sleep 2  # Give it a moment to fully initialize
            return 0
        fi
        log_info "Waiting... (${i}/10)"
        sleep 1
    done
    
    log_error "Failed to establish RFCOMM connection"
    return 1
}

# Function to set proper permissions
set_permissions() {
    log_info "Setting permissions for $RFCOMM_DEVICE..."
    
    if [ -e "$RFCOMM_DEVICE" ]; then
        $SUDO_CMD chmod 666 "$RFCOMM_DEVICE"
        
        # Verify permissions
        if [ -r "$RFCOMM_DEVICE" ] && [ -w "$RFCOMM_DEVICE" ]; then
            log_success "Permissions set successfully"
            return 0
        else
            log_error "Failed to set proper permissions"
            return 1
        fi
    else
        log_error "RFCOMM device $RFCOMM_DEVICE does not exist"
        return 1
    fi
}

# Function to test the connection
test_connection() {
    log_info "Testing Bluetooth connection..."
    
    if [ ! -e "$RFCOMM_DEVICE" ]; then
        log_error "RFCOMM device does not exist"
        return 1
    fi
    
    # Check if we can read from the device (with timeout)
    log_info "Testing data reception..."
    if timeout 5 cat "$RFCOMM_DEVICE" >/dev/null 2>&1; then
        log_success "Connection test successful - data is being received"
        return 0
    else
        log_warning "No data received in 5 seconds - connection may not be fully ready"
        log_info "This is normal if ESP32 is not sending data yet"
        return 0  # Don't fail here, ESP32 might just be idle
    fi
}

# Function to display connection status
show_status() {
    log_info "=== Bluetooth Crutch Connection Status ==="
    
    echo "Bluetooth service: $(systemctl is-active bluetooth)"
    echo "HCI0 interface: $(hciconfig hci0 | grep -o 'UP RUNNING' || echo 'DOWN')"
    
    if [ -e "$RFCOMM_DEVICE" ]; then
        echo "RFCOMM device: EXISTS ($(ls -la $RFCOMM_DEVICE))"
    else
        echo "RFCOMM device: MISSING"
    fi
    
    if $SUDO_CMD rfcomm show | grep -q "^rfcomm${RFCOMM_ID}"; then
        echo "RFCOMM connection: ACTIVE"
        $SUDO_CMD rfcomm show
    else
        echo "RFCOMM connection: INACTIVE"
    fi
    
    log_info "=============================================="
}

# Function to cleanup on script exit
cleanup_on_exit() {
    if [[ "$1" != "success" ]]; then
        log_warning "Script interrupted or failed, cleaning up..."
        cleanup_rfcomm
    fi
}

# Set trap for cleanup
trap 'cleanup_on_exit' EXIT INT TERM

# Main execution function
main() {
    log_info "Starting automated Bluetooth Smart Crutch setup..."
    
    # Parse command line arguments
    case "${1:-}" in
        "status"|"-s"|"--status")
            show_status
            exit 0
            ;;
        "cleanup"|"-c"|"--cleanup")
            cleanup_rfcomm
            log_success "Cleanup completed"
            exit 0
            ;;
        "help"|"-h"|"--help")
            echo "Usage: $SCRIPT_NAME [command]"
            echo "Commands:"
            echo "  (no args)  - Full setup (default)"
            echo "  status     - Show connection status"
            echo "  cleanup    - Cleanup existing connections"
            echo "  help       - Show this help"
            exit 0
            ;;
    esac
    
    # Check sudo availability
    check_sudo
    
    # Step 1: Check and start Bluetooth service
    if ! check_bluetooth_service; then
        log_error "Failed to setup Bluetooth service"
        exit 1
    fi
    
    # Step 2: Setup Bluetooth interface
    if ! setup_bluetooth_interface; then
        log_error "Failed to setup Bluetooth interface"
        exit 1
    fi
    
    # Step 3: Check for existing connections
    if check_existing_rfcomm; then
        log_info "Using existing RFCOMM connection"
    else
        # Step 4: Clean up any stale connections
        cleanup_rfcomm
        
        # Step 5: Establish new RFCOMM connection
        if ! establish_rfcomm_connection; then
            log_error "Failed to establish RFCOMM connection"
            exit 1
        fi
    fi
    
    # Step 6: Set proper permissions
    if ! set_permissions; then
        log_error "Failed to set permissions"
        exit 1
    fi
    
    # Step 7: Test the connection
    test_connection
    
    # Step 8: Show final status
    show_status
    
    log_success "Bluetooth Smart Crutch setup completed successfully!"
    log_info "You can now launch the ROS node with:"
    log_info "  roslaunch exoskeleton_control bluetooth_crutch.launch"
    
    # Don't cleanup on successful exit
    trap - EXIT INT TERM
}

# Run main function
main "$@"