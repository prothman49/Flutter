#!/bin/bash

# Script to analyze rosbag files

echo "======================================================================"
echo "ROSBAG ANALYZER"
echo "======================================================================"
echo ""

if [ -z "$1" ]; then
    echo "Usage: ./check_bags.sh <path_to_bag_file_or_directory>"
    echo ""
    echo "Examples:"
    echo "  ./check_bags.sh ~/catkin_ws/src/flutter_control/data/bags/"
    echo "  ./check_bags.sh my_test.bag"
    exit 1
fi

TARGET="$1"

# Function to analyze a single bag
analyze_bag() {
    BAG="$1"
    echo ""
    echo "======================================================================"
    echo "FILE: $(basename $BAG)"
    echo "======================================================================"
    echo ""
    
    # Get basic info
    echo "BASIC INFO:"
    echo "----------------------------------------------------------------------"
    rosbag info "$BAG"
    
    echo ""
    echo "TOPICS AVAILABLE:"
    echo "----------------------------------------------------------------------"
    rosbag info "$BAG" | grep "topics:" -A 100 | grep "^  " | awk '{print $1, $2, $3}'
    
    echo ""
    echo "======================================================================"
}

# Check if it's a directory or file
if [ -d "$TARGET" ]; then
    echo "Analyzing all .bag files in: $TARGET"
    echo ""
    
    # Find all bag files
    BAG_FILES=$(find "$TARGET" -name "*.bag" -type f)
    
    if [ -z "$BAG_FILES" ]; then
        echo "No .bag files found in $TARGET"
        exit 1
    fi
    
    COUNT=$(echo "$BAG_FILES" | wc -l)
    echo "Found $COUNT bag file(s)"
    echo ""
    
    # Analyze each bag
    for BAG in $BAG_FILES; do
        analyze_bag "$BAG"
    done
    
elif [ -f "$TARGET" ]; then
    analyze_bag "$TARGET"
else
    echo "Error: $TARGET is neither a file nor a directory"
    exit 1
fi

echo ""
echo "======================================================================"
echo "DONE"
echo "======================================================================"