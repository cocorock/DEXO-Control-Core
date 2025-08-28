#!/bin/bash

# Complete Bluetooth Smart Crutch Launch Script
# This script automatically sets up Bluetooth connection and launches the ROS node

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="$SCRIPT_DIR/setup_bluetooth_crutch.sh"
PACKAGE_NAME="exoskeleton_control"
LAUNCH_FILE="bluetooth_crutch.launch"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

LOG_PREFIX="[BT-CRUTCH-LAUNCH]"

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

# Function to check if ROS is available
check_ros_environment() {
    log_info "Checking ROS environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS environment not sourced"
        log_info "Please run: source /opt/ros/noetic/setup.bash"
        log_info "And: source /path/to/your/catkin_ws/devel/setup.bash"
        exit 1
    fi
    
    log_success "ROS environment detected: $ROS_DISTRO"
}

# Function to check if the package exists
check_package() {
    log_info "Checking if package '$PACKAGE_NAME' is available..."
    
    if ! rospack find "$PACKAGE_NAME" >/dev/null 2>&1; then
        log_error "Package '$PACKAGE_NAME' not found"
        log_info "Please build your catkin workspace first"
        exit 1
    fi
    
    local package_path=$(rospack find "$PACKAGE_NAME")
    log_success "Package found at: $package_path"
}

# Function to run Bluetooth setup
setup_bluetooth() {
    log_info "Setting up Bluetooth connection..."
    
    if [ ! -x "$SETUP_SCRIPT" ]; then
        log_error "Setup script not found or not executable: $SETUP_SCRIPT"
        exit 1
    fi
    
    # Run the setup script
    if "$SETUP_SCRIPT"; then
        log_success "Bluetooth setup completed successfully"
        return 0
    else
        log_error "Bluetooth setup failed"
        return 1
    fi
}

# Function to launch ROS node
launch_ros_node() {
    log_info "Launching ROS node..."
    log_info "Command: roslaunch $PACKAGE_NAME $LAUNCH_FILE"
    
    # Add a small delay to ensure Bluetooth connection is fully ready
    sleep 2
    
    # Launch with exec to replace this process and handle signals properly
    exec roslaunch "$PACKAGE_NAME" "$LAUNCH_FILE"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --setup-only, -s    Only run Bluetooth setup, don't launch ROS"
    echo "  --skip-setup        Skip Bluetooth setup, launch ROS directly"
    echo "  --status            Show Bluetooth connection status"
    echo "  --cleanup           Cleanup Bluetooth connections"
    echo "  --help, -h          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                  # Full automated launch (setup + ROS)"
    echo "  $0 --setup-only     # Only setup Bluetooth"
    echo "  $0 --status         # Check connection status"
    echo "  $0 --cleanup        # Cleanup and exit"
}

# Function to handle cleanup on exit
cleanup_on_exit() {
    log_info "Script interrupted, cleaning up..."
    # Kill any background processes if needed
    pkill -f "rfcomm connect" || true
}

# Set trap for cleanup
trap cleanup_on_exit INT TERM

# Main function
main() {
    log_info "=== Bluetooth Smart Crutch Launcher ==="
    
    # Parse command line arguments
    case "${1:-}" in
        "--setup-only"|"-s")
            log_info "Running setup only mode"
            setup_bluetooth
            log_success "Setup completed. You can now launch ROS manually."
            exit 0
            ;;
        "--skip-setup")
            log_info "Skipping Bluetooth setup"
            check_ros_environment
            check_package
            launch_ros_node
            ;;
        "--status")
            "$SETUP_SCRIPT" status
            exit 0
            ;;
        "--cleanup")
            "$SETUP_SCRIPT" cleanup
            exit 0
            ;;
        "--help"|"-h"|"help")
            show_usage
            exit 0
            ;;
        "")
            # Default: full automated launch
            log_info "Running full automated launch"
            ;;
        *)
            log_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
    
    # Full automated launch sequence
    log_info "Starting full automated launch sequence..."
    
    # Step 1: Check ROS environment
    check_ros_environment
    
    # Step 2: Check package availability
    check_package
    
    # Step 3: Setup Bluetooth connection
    if ! setup_bluetooth; then
        log_error "Failed to setup Bluetooth connection"
        log_info "You can try:"
        log_info "  1. Check if ESP32 is powered on and Bluetooth enabled"
        log_info "  2. Run '$0 --status' to check current status"
        log_info "  3. Run '$0 --cleanup' to clean up and try again"
        exit 1
    fi
    
    # Step 4: Launch ROS node
    log_success "All setup complete, launching ROS node..."
    launch_ros_node
}

# Run main function
main "$@"