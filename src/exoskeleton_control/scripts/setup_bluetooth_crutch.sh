#!/bin/bash

# Automated Bluetooth Smart Crutch Setup Script
# This script handles the complete Bluetooth connection setup for the smart crutch

set -e  # Exit on any error

# Configuration - Dual Crutch Support
ESP32_RIGHT_MAC="24:6F:28:D1:36:72"    # Right crutch (priority device)
ESP32_LEFT_MAC="24:6f:28:45:d3:76"     # Left crutch
RFCOMM_CHANNEL="1"
RFCOMM_RIGHT_DEVICE="/dev/rfcomm0"
RFCOMM_LEFT_DEVICE="/dev/rfcomm1"
RFCOMM_RIGHT_ID="0"
RFCOMM_LEFT_ID="1"
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

# Function to check if RFCOMM connection already exists (dual crutch)
check_existing_rfcomm() {
    local right_exists=false
    local left_exists=false
    
    # Check right crutch
    if [ -e "$RFCOMM_RIGHT_DEVICE" ]; then
        log_info "RFCOMM device $RFCOMM_RIGHT_DEVICE (right) already exists"
        if [ -r "$RFCOMM_RIGHT_DEVICE" ] && [ -w "$RFCOMM_RIGHT_DEVICE" ]; then
            log_success "Existing right crutch RFCOMM connection appears to be working"
            right_exists=true
        else
            log_warning "Right crutch RFCOMM device exists but may not be functional"
        fi
    fi
    
    # Check left crutch
    if [ -e "$RFCOMM_LEFT_DEVICE" ]; then
        log_info "RFCOMM device $RFCOMM_LEFT_DEVICE (left) already exists"
        if [ -r "$RFCOMM_LEFT_DEVICE" ] && [ -w "$RFCOMM_LEFT_DEVICE" ]; then
            log_success "Existing left crutch RFCOMM connection appears to be working"
            left_exists=true
        else
            log_warning "Left crutch RFCOMM device exists but may not be functional"
        fi
    fi
    
    # Return success only if both connections exist and work
    if $right_exists && $left_exists; then
        return 0
    else
        # Clean up non-working connections
        cleanup_rfcomm
        return 1
    fi
}

# Function to cleanup existing RFCOMM connections (dual crutch)
cleanup_rfcomm() {
    log_info "Cleaning up existing RFCOMM connections (dual crutch)..."
    
    # Release right crutch connection
    if $SUDO_CMD rfcomm show $RFCOMM_RIGHT_ID 2>/dev/null | grep -q "rfcomm$RFCOMM_RIGHT_ID"; then
        log_info "Releasing existing right crutch RFCOMM connection..."
        $SUDO_CMD rfcomm release $RFCOMM_RIGHT_ID || true
        sleep 1
    fi
    
    # Release left crutch connection  
    if $SUDO_CMD rfcomm show $RFCOMM_LEFT_ID 2>/dev/null | grep -q "rfcomm$RFCOMM_LEFT_ID"; then
        log_info "Releasing existing left crutch RFCOMM connection..."
        $SUDO_CMD rfcomm release $RFCOMM_LEFT_ID || true
        sleep 1
    fi
    
    # Remove device files if they exist
    if [ -e "$RFCOMM_RIGHT_DEVICE" ]; then
        log_info "Removing existing right crutch RFCOMM device file..."
        $SUDO_CMD rm -f "$RFCOMM_RIGHT_DEVICE" || true
    fi
    
    if [ -e "$RFCOMM_LEFT_DEVICE" ]; then
        log_info "Removing existing left crutch RFCOMM device file..."
        $SUDO_CMD rm -f "$RFCOMM_LEFT_DEVICE" || true
    fi
}

# Function to establish RFCOMM connections (dual crutch)
establish_rfcomm_connection() {
    local right_success=false
    local left_success=false
    
    log_info "Establishing dual RFCOMM connections..."
    
    # Check if ESP32 devices are discoverable/paired
    log_info "Scanning for ESP32 devices..."
    if ! hcitool scan | grep -q "$ESP32_RIGHT_MAC"; then
        log_warning "Right crutch ESP32 device $ESP32_RIGHT_MAC not found in scan"
        log_info "Make sure the right crutch ESP32 is powered on and Bluetooth is enabled"
    fi
    
    if ! hcitool scan | grep -q "$ESP32_LEFT_MAC"; then
        log_warning "Left crutch ESP32 device $ESP32_LEFT_MAC not found in scan"
        log_info "Make sure the left crutch ESP32 is powered on and Bluetooth is enabled"
    fi
    
    # Bind RFCOMM devices (this creates the device files without holding them open)
    log_info "Binding right crutch ESP32 via RFCOMM..."
    $SUDO_CMD rfcomm bind $RFCOMM_RIGHT_ID $ESP32_RIGHT_MAC $RFCOMM_CHANNEL
    
    log_info "Binding left crutch ESP32 via RFCOMM..."
    $SUDO_CMD rfcomm bind $RFCOMM_LEFT_ID $ESP32_LEFT_MAC $RFCOMM_CHANNEL
    
    # Wait for device files to be created
    log_info "Waiting for RFCOMM device files to be created..."
    sleep 2  # Give time for device files to appear
    
    # Check if device files were created
    if [ -e "$RFCOMM_RIGHT_DEVICE" ]; then
        log_success "Right crutch RFCOMM device created"
        right_success=true
    else
        log_error "Right crutch RFCOMM device not created"
    fi
    
    if [ -e "$RFCOMM_LEFT_DEVICE" ]; then
        log_success "Left crutch RFCOMM device created"
        left_success=true
    else
        log_error "Left crutch RFCOMM device not created"
    fi
    
    # Report results
    if $right_success && $left_success; then
        log_success "Both RFCOMM devices established successfully"
        return 0
    elif $right_success && ! $left_success; then
        log_warning "Only right crutch device established"
        return 2  # Partial success
    elif ! $right_success && $left_success; then
        log_warning "Only left crutch device established"  
        return 2  # Partial success
    else
        log_error "Failed to establish RFCOMM devices"
        return 1  # Complete failure
    fi
}

# Function to set proper permissions (dual crutch)
set_permissions() {
    log_info "Setting permissions for dual crutch devices..."
    local right_success=false
    local left_success=false
    
    # Set permissions for right crutch
    if [ -e "$RFCOMM_RIGHT_DEVICE" ]; then
        $SUDO_CMD chmod 666 "$RFCOMM_RIGHT_DEVICE"
        
        if [ -r "$RFCOMM_RIGHT_DEVICE" ] && [ -w "$RFCOMM_RIGHT_DEVICE" ]; then
            log_success "Right crutch permissions set successfully"
            right_success=true
        else
            log_error "Failed to set proper permissions for right crutch"
        fi
    else
        log_error "Right crutch RFCOMM device $RFCOMM_RIGHT_DEVICE does not exist"
    fi
    
    # Set permissions for left crutch
    if [ -e "$RFCOMM_LEFT_DEVICE" ]; then
        $SUDO_CMD chmod 666 "$RFCOMM_LEFT_DEVICE"
        
        if [ -r "$RFCOMM_LEFT_DEVICE" ] && [ -w "$RFCOMM_LEFT_DEVICE" ]; then
            log_success "Left crutch permissions set successfully"
            left_success=true
        else
            log_error "Failed to set proper permissions for left crutch"
        fi
    else
        log_error "Left crutch RFCOMM device $RFCOMM_LEFT_DEVICE does not exist"
    fi
    
    # Return success if at least one device has proper permissions
    if $right_success || $left_success; then
        return 0
    else
        return 1
    fi
}

# Function to test the connections (dual crutch)
test_connection() {
    log_info "Testing dual Bluetooth connections..."
    local right_tested=false
    local left_tested=false
    
    # Test right crutch connection
    if [ -e "$RFCOMM_RIGHT_DEVICE" ]; then
        log_info "Testing right crutch data reception..."
        if timeout 3 cat "$RFCOMM_RIGHT_DEVICE" >/dev/null 2>&1; then
            log_success "Right crutch connection test successful - data is being received"
        else
            log_warning "Right crutch: No data received in 3 seconds - connection may not be fully ready"
        fi
        right_tested=true
    else
        log_error "Right crutch RFCOMM device does not exist"
    fi
    
    # Test left crutch connection
    if [ -e "$RFCOMM_LEFT_DEVICE" ]; then
        log_info "Testing left crutch data reception..."
        if timeout 3 cat "$RFCOMM_LEFT_DEVICE" >/dev/null 2>&1; then
            log_success "Left crutch connection test successful - data is being received"
        else
            log_warning "Left crutch: No data received in 3 seconds - connection may not be fully ready"
        fi
        left_tested=true
    else
        log_error "Left crutch RFCOMM device does not exist"
    fi
    
    if $right_tested || $left_tested; then
        log_info "This is normal if ESP32 devices are not sending data yet"
        return 0  # Don't fail here, ESP32s might just be idle
    else
        return 1
    fi
}

# Function to display connection status (dual crutch)
show_status() {
    log_info "=== Dual Bluetooth Crutch Connection Status ==="
    
    echo "Bluetooth service: $(systemctl is-active bluetooth)"
    echo "HCI0 interface: $(hciconfig hci0 | grep -o 'UP RUNNING' || echo 'DOWN')"
    echo ""
    
    # Right crutch status
    echo "=== RIGHT CRUTCH ($ESP32_RIGHT_MAC) ==="
    if [ -e "$RFCOMM_RIGHT_DEVICE" ]; then
        echo "RFCOMM device: EXISTS ($(ls -la $RFCOMM_RIGHT_DEVICE))"
    else
        echo "RFCOMM device: MISSING"
    fi
    
    if $SUDO_CMD rfcomm show $RFCOMM_RIGHT_ID 2>/dev/null | grep -q "rfcomm$RFCOMM_RIGHT_ID"; then
        echo "RFCOMM connection: ACTIVE"
    else
        echo "RFCOMM connection: INACTIVE"
    fi
    echo ""
    
    # Left crutch status
    echo "=== LEFT CRUTCH ($ESP32_LEFT_MAC) ==="
    if [ -e "$RFCOMM_LEFT_DEVICE" ]; then
        echo "RFCOMM device: EXISTS ($(ls -la $RFCOMM_LEFT_DEVICE))"
    else
        echo "RFCOMM device: MISSING"
    fi
    
    if $SUDO_CMD rfcomm show $RFCOMM_LEFT_ID 2>/dev/null | grep -q "rfcomm$RFCOMM_LEFT_ID"; then
        echo "RFCOMM connection: ACTIVE"
    else
        echo "RFCOMM connection: INACTIVE"
    fi
    echo ""
    
    # Show all RFCOMM connections
    echo "=== ALL RFCOMM CONNECTIONS ==="
    $SUDO_CMD rfcomm 2>/dev/null || echo "No active RFCOMM connections"
    
    log_info "================================================"
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
    
    log_success "Dual Bluetooth Smart Crutch setup completed successfully!"
    log_info "You can now launch the ROS node with:"
    log_info "  roslaunch exoskeleton_control bluetooth_crutch.launch"
    log_info "Devices configured:"
    log_info "  Right crutch: $ESP32_RIGHT_MAC -> $RFCOMM_RIGHT_DEVICE"
    log_info "  Left crutch:  $ESP32_LEFT_MAC -> $RFCOMM_LEFT_DEVICE"
    
    # Don't cleanup on successful exit
    trap - EXIT INT TERM
}

# Run main function
main "$@"