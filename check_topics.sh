#!/bin/bash
# Topic Health Check Script for 3D Ocean Exploration System
# This script verifies all mapping and frontier detection topics are working

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "============================================"
echo "  3D Ocean Exploration - Topic Health Check"
echo "============================================"
echo ""

# Function to check if a topic exists and is publishing
check_topic() {
    local topic=$1
    local expected_rate=$2
    local timeout=${3:-3}

    echo -n "Checking ${topic}... "

    # Check if topic exists
    if ! ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${RED}✗ NOT FOUND${NC}"
        return 1
    fi

    # Check if topic is publishing
    if timeout ${timeout} ros2 topic hz ${topic} 2>&1 | grep -q "average rate"; then
        echo -e "${GREEN}✓ Publishing${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠ Exists but not publishing${NC}"
        return 2
    fi
}

# Function to show topic content
show_topic_sample() {
    local topic=$1
    local lines=${2:-15}

    echo -e "\n${BLUE}Sample from ${topic}:${NC}"
    timeout 2 ros2 topic echo ${topic} --once 2>/dev/null | head -n ${lines} || echo "  (No data)"
}

echo "=== 1. Input Topics (from simulation) ==="
check_topic "/UW_Camera_Stereo_pointcloud" "10-30 Hz" 5
check_topic "/tf" "high" 3
check_topic "/clock" "high" 3
echo ""

echo "=== 2. Mapping Topics (octomap_3d_exploration) ==="
check_topic "/octomap_binary" "~0.4 Hz" 5
check_topic "/octomap_full" "~0.4 Hz" 5
check_topic "/occupied_cells_vis" "~0.4 Hz" 5
check_topic "/free_cells_vis" "~0.4 Hz" 5
echo ""

echo "=== 3. Frontier Topics (frontier_detection_3d) ⭐ ==="
check_topic "/exploration_goal" "1 Hz" 5
check_topic "/frontier_markers" "1 Hz" 5
check_topic "/best_frontier_marker" "1 Hz" 5
echo ""

echo "=== 4. System Topics ==="
check_topic "/parameter_events" "low" 3
check_topic "/rosout" "low" 3
echo ""

# Check active nodes
echo "=== 5. Active Nodes ==="
echo -e "${BLUE}Running nodes:${NC}"
ros2 node list | while read node; do
    echo "  ✓ ${node}"
done
echo ""

# Show exploration goal if available
if ros2 topic list | grep -q "^/exploration_goal$"; then
    show_topic_sample "/exploration_goal" 20
fi

# Show frontier statistics from logs (if available)
echo ""
echo "=== 6. Frontier Detection Status ==="
echo -e "${BLUE}Recent frontier detection logs:${NC}"
echo "(Check your terminal where frontier_detector is running)"
echo ""

echo "============================================"
echo "  Health Check Complete"
echo "============================================"
echo ""
echo "📋 Next Steps:"
echo "  1. All topics green (✓)? System is working!"
echo "  2. Topics not publishing (⚠)? Wait 10-20 seconds for map to build"
echo "  3. Topics not found (✗)? Launch the corresponding node"
echo ""
echo "💡 Tips:"
echo "  - View in RViz: ros2 launch octomap_3d_exploration visualization.launch.py"
echo "  - Monitor frontiers: ros2 topic echo /exploration_goal"
echo "  - Check logs: Look at the terminals running the nodes"
echo ""
