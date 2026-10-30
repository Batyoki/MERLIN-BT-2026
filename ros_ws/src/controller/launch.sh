#!/bin/bash
# Quick launch commands for bilateral teleoperation with simulated master

set -e

WORKSPACE="/home/sp/Desktop/6th_Semester/RnD/ws"
CONTROLLER_PKG="controller"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

print_info() {
    echo -e "${YELLOW}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

case "${1:-}" in
    sim_only)
        print_header "Launching: Simulated Master Only"
        print_info "This runs just the simulated rx150 master for testing"
        print_info "No real hardware required"
        print_info ""
        print_info "Joystick optional - you can test with:"
        print_info "  ros2 run joy joy_node  (in another terminal)"
        cd $WORKSPACE
        source install/setup.bash
        ros2 launch $CONTROLLER_PKG sim_master_only.launch.py
        ;;
    
    keyboard)
        print_header "Launching: Simulated Master with Keyboard Control"
        print_info "Use keyboard to control the simulated rx150 master"
        print_info "A/D = Waist, W/S = Shoulder, Q/E = Elbow"
        print_info "R/F = Wrist Angle, Z/X = Wrist Rotate"
        print_info "UP/DOWN = Speed, SPACE = Stop, H = Help"
        cd $WORKSPACE
        source install/setup.bash
        ros2 launch $CONTROLLER_PKG sim_master_keyboard.launch.py
        ;;
    
    sim_real)
        print_header "Launching: Simulated Master + Real Slave"
        print_info "This runs the full bilateral system:"
        print_info "  - Simulated master (rx150) on desktop"
        print_info "  - Real slave (vx300s) via InterbotiX"
        print_info ""
        print_info "Make sure your slave arm is powered on and connected!"
        cd $WORKSPACE
        source install/setup.bash
        ros2 launch $CONTROLLER_PKG sim_master_real_slave.launch.py
        ;;
    
    nodes)
        print_header "Launching: Individual Nodes (Manual)"
        print_info "Start each node in a separate terminal:"
        echo ""
        echo "Terminal 1 - Simulated Master:"
        echo "  ros2 run $CONTROLLER_PKG simulated_master"
        echo ""
        echo "Terminal 2 - Master State Node:"
        echo "  ros2 run $CONTROLLER_PKG master_state_node"
        echo ""
        echo "Terminal 3 - Slave State Node (if using real hardware):"
        echo "  ros2 run $CONTROLLER_PKG slave_state_node"
        echo ""
        echo "Terminal 4 - Controller:"
        echo "  ros2 run $CONTROLLER_PKG controller"
        echo ""
        echo "Terminal 5 - Joystick (optional):"
        echo "  ros2 run joy joy_node"
        echo ""
        echo "Terminal 6 - Joystick Teleop:"
        echo "  ros2 run $CONTROLLER_PKG joystick_teleop"
        ;;
    
    monitor)
        print_header "Monitoring Joint States"
        print_info "Simulated Master State:"
        cd $WORKSPACE
        source install/setup.bash
        ros2 topic echo /master_state_L --once
        ;;
    
    rebuild)
        print_header "Rebuilding Controller Package"
        cd $WORKSPACE
        colcon build --packages-select $CONTROLLER_PKG
        print_info "Build complete!"
        ;;
    
    help|"")
        print_header "Bilateral Teleoperation - Quick Launch"
        echo ""
        echo "Usage: $0 {command}"
        echo ""
        echo "Commands:"
        echo "  sim_only       - Launch simulated master only (no hardware)"
        echo "  keyboard       - Launch simulated master with keyboard control"
        echo "  sim_real       - Launch simulated master + real slave"
        echo "  nodes          - Show how to launch nodes individually"
        echo "  monitor        - Show current joint states"
        echo "  rebuild        - Rebuild the controller package"
        echo "  help           - Show this help message"
        echo ""
        echo "Quick Start:"
        echo "  1. For testing without hardware (keyboard control):"
        echo "     $0 keyboard"
        echo ""
        echo "  2. For testing without hardware (joystick):"
        echo "     $0 sim_only"
        echo ""
        echo "  3. For full bilateral system with real slave:"
        echo "     $0 sim_real"
        echo ""
        ;;
    
    *)
        print_error "Unknown command: $1"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac
