#!/bin/bash

# Automated Validation Testing Script
# Runs multiple validation tests at specified phases with automatic recording and organization

# Configuration
PHASES=(45 45 45 270 270)  # Run 3 tests at 45°, then 2 tests at 270°
CYCLE_DURATION=120
NUM_CYCLES=3
BASE_DIR="$HOME/catkin_ws/src/flutter_control"

echo "=========================================="
echo "AUTOMATED VALIDATION TEST SUITE"
echo "=========================================="
echo "Phases to test: ${PHASES[@]}"
echo "Each test: $NUM_CYCLES cycles × ${CYCLE_DURATION}s = ~12 minutes"
echo "Total estimated time: $((${#PHASES[@]} * 12)) minutes"
echo ""
echo "Press ENTER to start, or Ctrl+C to cancel"
read

# Function to run a single test
run_test() {
    local PHASE=$1
    local RUN_NUM=$2
    local TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    
    echo ""
    echo "=========================================="
    echo "TEST: Phase ${PHASE}° - Run ${RUN_NUM}"
    echo "Time: $(date)"
    echo "=========================================="
    
    # Create directories
    PHASE_DIR="$BASE_DIR/data/results/validation_bad_phases/phase_$(printf '%03d' $PHASE)deg"
    mkdir -p "$PHASE_DIR/json"
    mkdir -p "$PHASE_DIR/bags"
    
    # Start bag recording in background
    echo "[1/4] Starting bag recording..."
    roslaunch flutter_control record_experiment.launch \
        experiment_name:=validation_${PHASE}deg_run${RUN_NUM} \
        > /tmp/rosbag_${PHASE}_${RUN_NUM}.log 2>&1 &
    
    RECORD_PID=$!
    echo "Recording PID: $RECORD_PID"
    
    # Wait for recording to initialize
    sleep 5
    
    # Check if recording started successfully
    if ! ps -p $RECORD_PID > /dev/null; then
        echo "❌ ERROR: Recording failed to start!"
        cat /tmp/rosbag_${PHASE}_${RUN_NUM}.log
        return 1
    fi
    
    echo "✅ Recording started successfully"
    
    # Run validation test
    echo "[2/4] Running validation test (this will take ~12 minutes)..."
    cd "$BASE_DIR/scripts"
    
    # Create temporary expect-like script
    cat > /tmp/auto_input_${PHASE}_${RUN_NUM}.txt << EOF
${PHASE}
${CYCLE_DURATION}
${NUM_CYCLES}
EOF
    
    # Run test with automatic input
    python3 test_control_effectiveness.py < /tmp/auto_input_${PHASE}_${RUN_NUM}.txt
    
    TEST_EXIT_CODE=$?
    
    if [ $TEST_EXIT_CODE -ne 0 ]; then
        echo "⚠ WARNING: Test may have had issues (exit code: $TEST_EXIT_CODE)"
    fi
    
    # Stop bag recording
    echo "[3/4] Stopping bag recording..."
    kill -SIGINT $RECORD_PID
    sleep 3
    
    # Wait for recording to finish writing
    wait $RECORD_PID 2>/dev/null
    
    echo "✅ Recording stopped"
    
    # Move and organize files
    echo "[4/4] Organizing files..."
    
    # Find and move JSON file (most recent)
    LATEST_JSON=$(ls -t "$BASE_DIR/data/results/control_effectiveness_"*.json 2>/dev/null | head -1)
    if [ -f "$LATEST_JSON" ]; then
        mv "$LATEST_JSON" "$PHASE_DIR/json/run${RUN_NUM}_results.json"
        echo "✅ Moved JSON: run${RUN_NUM}_results.json"
    else
        echo "❌ WARNING: No JSON file found!"
    fi
    
    # Find and move bag file
    LATEST_BAG=$(ls -t "$BASE_DIR/data/results/bags/validation_${PHASE}deg_run${RUN_NUM}_"*.bag 2>/dev/null | head -1)
    if [ -f "$LATEST_BAG" ]; then
        mv "$LATEST_BAG" "$PHASE_DIR/bags/run${RUN_NUM}_data.bag"
        
        # Get bag info
        BAG_SIZE=$(ls -lh "$PHASE_DIR/bags/run${RUN_NUM}_data.bag" | awk '{print $5}')
        echo "✅ Moved BAG: run${RUN_NUM}_data.bag (Size: $BAG_SIZE)"
        
        # Verify bag has piezo data
        if rosbag info "$PHASE_DIR/bags/run${RUN_NUM}_data.bag" 2>/dev/null | grep -q "piezo_voltages"; then
            echo "✅ Verified: Bag contains piezo_voltages"
        else
            echo "⚠ WARNING: Bag missing piezo_voltages!"
        fi
    else
        echo "❌ WARNING: No bag file found!"
    fi
    
    # Cleanup temp files
    rm -f /tmp/auto_input_${PHASE}_${RUN_NUM}.txt
    rm -f /tmp/rosbag_${PHASE}_${RUN_NUM}.log
    
    echo ""
    echo "✅ Test ${PHASE}° Run ${RUN_NUM} COMPLETE"
    echo "Files saved to: $PHASE_DIR"
    echo ""
    
    # Wait between tests (allow flutter to settle)
    if [ $RUN_NUM -lt ${#PHASES[@]} ]; then
        echo "Waiting 30 seconds before next test..."
        sleep 30
    fi
}

# Main execution
echo ""
echo "Starting automated test suite..."
echo ""

# Track run numbers for each phase
declare -A RUN_COUNTS

# Run all tests
for i in "${!PHASES[@]}"; do
    PHASE=${PHASES[$i]}
    
    # Increment run count for this phase
    if [ -z "${RUN_COUNTS[$PHASE]}" ]; then
        RUN_COUNTS[$PHASE]=1
    else
        RUN_COUNTS[$PHASE]=$((${RUN_COUNTS[$PHASE]} + 1))
    fi
    
    RUN_NUM=${RUN_COUNTS[$PHASE]}
    TEST_NUM=$((i + 1))
    
    echo ""
    echo "=========================================="
    echo "PROGRESS: Test $TEST_NUM of ${#PHASES[@]}"
    echo "=========================================="
    
    run_test $PHASE $RUN_NUM
    
    if [ $? -ne 0 ]; then
        echo ""
        echo "❌ Test failed! Do you want to continue? (y/n)"
        read -t 30 CONTINUE
        if [ "$CONTINUE" != "y" ]; then
            echo "Aborting test suite..."
            exit 1
        fi
    fi
done

# Final summary
echo ""
echo "=========================================="
echo "ALL TESTS COMPLETE!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  Total tests run: ${#PHASES[@]}"
echo "  Phases tested: 45° (3 runs), 270° (2 runs)"
echo ""
echo "Results location:"
echo "  $BASE_DIR/data/results/validation_bad_phases/"
echo ""
echo "Next steps:"
echo "  1. Review JSON results"
echo "  2. Verify bag files"
echo "  3. Analyze data"
echo ""