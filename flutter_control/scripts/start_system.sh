#!/bin/bash

# Flutter Control System Startup Script
# This starts camera, optic flow, and optionally the control node

set -e  # Exit on error

echo "=========================================="
echo "Flutter Control System Startup"
echo "=========================================="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if roscore is running
if ! pgrep -x "roscore" > /dev/null; then
    echo -e "${YELLOW}Starting roscore...${NC}"
    roscore &
    sleep 3
fi

echo -e "${GREEN}✓ roscore running${NC}"

# Start camera
echo -e "${YELLOW}Starting camera node...${NC}"
roslaunch pylon_camera pylon_camera_node.launch &
CAMERA_PID=$!
sleep 3

echo -e "${GREEN}✓ Camera started${NC}"

# Start optic flow
echo -e "${YELLOW}Starting optic flow node...${NC}"
rosrun optic_flow_example optic_flow_lucas_kanade.py --topic=/pylon_camera_node/image_raw &
OPTIC_FLOW_PID=$!
sleep 2

echo -e "${GREEN}✓ Optic flow started${NC}"

# Check if optic flow topic is publishing
echo "Checking optic flow topic..."
if rostopic echo /optic_flow_mean_vx -n 1 --timeout=5 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Optic flow data available${NC}"
else
    echo -e "${YELLOW}⚠ Warning: No optic flow data detected${NC}"
fi

echo ""
echo "=========================================="
echo "System Ready!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Run a test script:"
echo "     python3 scripts/test_structural_phase.py"
echo "     python3 scripts/test_flutter_phase_sweep.py"
echo "     python3 scripts/run_live_control.py"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "=========================================="

# Wait for user interrupt
trap "echo ''; echo 'Shutting down...'; kill $CAMERA_PID $OPTIC_FLOW_PID 2>/dev/null; exit" INT

wait