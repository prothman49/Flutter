#!/bin/bash

# Usage: ./organize_sweep.sh <run_number>

RUN=$1

if [ -z "$RUN" ]; then
    echo "Usage: ./organize_sweep.sh <run_number>"
    exit 1
fi

BASE_DIR="$HOME/catkin_ws/src/flutter_control/data/results"

mkdir -p "$BASE_DIR/phase_sweeps/coarse_sweeps/json"
mkdir -p "$BASE_DIR/phase_sweeps/coarse_sweeps/bags"

# Move JSON
LATEST_JSON=$(ls -t $BASE_DIR/flutter_phase_sweep_*.json 2>/dev/null | head -1)
if [ -f "$LATEST_JSON" ]; then
    mv "$LATEST_JSON" "$BASE_DIR/phase_sweeps/coarse_sweeps/json/sweep_run${RUN}.json"
    echo "✅ Moved JSON: sweep_run${RUN}.json"
else
    echo "❌ No JSON file found"
fi

# Move bag
LATEST_BAG=$(ls -t $BASE_DIR/bags/coarse_sweep_run${RUN}_*.bag 2>/dev/null | head -1)
if [ -f "$LATEST_BAG" ]; then
    mv "$LATEST_BAG" "$BASE_DIR/phase_sweeps/coarse_sweeps/bags/sweep_run${RUN}.bag"
    echo "✅ Moved BAG: sweep_run${RUN}.bag"
    
    # Quick info
    rosbag info "$BASE_DIR/phase_sweeps/coarse_sweeps/bags/sweep_run${RUN}.bag" | grep -E "duration|piezo_voltages"
else
    echo "❌ No bag file found"
fi

echo ""
echo "Files organized in: $BASE_DIR/phase_sweeps/coarse_sweeps/"