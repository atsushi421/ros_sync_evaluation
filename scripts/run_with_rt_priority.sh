#!/bin/bash

set -e # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running with sufficient privileges
if [ "$EUID" -ne 0 ]; then
	echo -e "${RED}Error: This script must be run with sudo or as root${NC}"
	echo "Usage: sudo $0 <num_publishers> <install_path>"
	exit 1
fi

# Parse arguments
if [ $# -lt 2 ]; then
	echo -e "${RED}Error: Missing required arguments${NC}"
	echo ""
	echo "Usage: sudo $0 <num_publishers> <install_path>"
	echo ""
	echo "Arguments:"
	echo "  num_publishers: Number of publishers to launch (1-4)"
	echo "  install_path: Path to ROS 2 installation directory"
	echo ""
	echo "Example:"
	echo "  sudo $0 3 /home/atsushi/ros_sync_evaluation/install/ros_sync_evaluation"
	exit 1
fi

NUM_PUBLISHERS="$1"
INSTALL_PATH="$2"

# Validate number of publishers
if ! [[ "$NUM_PUBLISHERS" =~ ^[1-4]$ ]]; then
	echo -e "${RED}Error: num_publishers must be 1, 2, 3, or 4${NC}"
	exit 1
fi

# Check if install path exists
if [ ! -d "$INSTALL_PATH" ]; then
	echo -e "${RED}Error: Install path not found: $INSTALL_PATH${NC}"
	exit 1
fi

# Set executable paths
SOURCE_PUBLISHER="$INSTALL_PATH/lib/ros_sync_evaluation/source_publisher"
SUBSCRIBE_PUBLISHER="$INSTALL_PATH/lib/ros_sync_evaluation/subscribe_publisher"
SYNC_SUBSCRIBER="$INSTALL_PATH/lib/ros_sync_evaluation/sync_subscriber"

# Check if executables exist
if [ ! -f "$SOURCE_PUBLISHER" ]; then
	echo -e "${RED}Error: source_publisher not found at $SOURCE_PUBLISHER${NC}"
	exit 1
fi

if [ ! -f "$SUBSCRIBE_PUBLISHER" ]; then
	echo -e "${RED}Error: subscribe_publisher not found at $SUBSCRIBE_PUBLISHER${NC}"
	exit 1
fi

if [ ! -f "$SYNC_SUBSCRIBER" ]; then
	echo -e "${RED}Error: sync_subscriber not found at $SYNC_SUBSCRIBER${NC}"
	exit 1
fi

# Check if chrt is available
if ! command -v chrt &>/dev/null; then
	echo -e "${RED}Error: 'chrt' command not found${NC}"
	echo "Install util-linux package: sudo apt-get install util-linux"
	exit 1
fi

# Set PMU analyzer config file if not already set
if [ -z "$PMU_ANALYZER_CONFIG_FILE" ]; then
	CONFIG_FILE="/home/atsushi/ros_sync_evaluation/config/pmu_config.yaml"

	if [ -f "$CONFIG_FILE" ]; then
		export PMU_ANALYZER_CONFIG_FILE="$CONFIG_FILE"
		echo -e "${GREEN}Set PMU_ANALYZER_CONFIG_FILE to: $CONFIG_FILE${NC}"
	else
		echo -e "${YELLOW}Warning: PMU config file not found at $CONFIG_FILE${NC}"
		echo -e "${YELLOW}Set PMU_ANALYZER_CONFIG_FILE environment variable if needed${NC}"
	fi
fi

# Display configuration
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ROS 2 Sync Evaluation Launcher${NC}"
echo -e "${GREEN}========================================${NC}"
echo "  Number of Publishers: $NUM_PUBLISHERS"
echo "  Install Path: $INSTALL_PATH"
echo "  Scheduling Policy: SCHED_FIFO (subscriber only)"
echo "  Priority: 99 (subscriber only)"
echo "  PMU Config: ${PMU_ANALYZER_CONFIG_FILE:-<not set>}"
echo ""

# Array to store background process PIDs
declare -a PIDS

# Cleanup function to kill all background processes
cleanup() {
	echo ""
	echo -e "${YELLOW}Shutting down all nodes...${NC}"
	for pid in "${PIDS[@]}"; do
		if kill -0 "$pid" 2>/dev/null; then
			kill "$pid" 2>/dev/null || true
		fi
	done
	wait
	echo -e "${GREEN}All nodes stopped${NC}"
	exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Launch source_publisher
echo -e "${BLUE}Launching source_publisher...${NC}"
"$SOURCE_PUBLISHER" &
PIDS+=($!)
echo -e "${GREEN}  PID: ${PIDS[-1]}${NC}"

# Launch subscribe_publisher instances
for ((i = 1; i <= NUM_PUBLISHERS; i++)); do
	echo -e "${BLUE}Launching subscribe_publisher (topic_id=$i)...${NC}"
	"$SUBSCRIBE_PUBLISHER" --ros-args -p topic_id:="$i" &
	PIDS+=($!)
	echo -e "${GREEN}  PID: ${PIDS[-1]}${NC}"
done

# Launch sync_subscriber with real-time priority
echo -e "${BLUE}Launching sync_subscriber with RT priority...${NC}"
chrt -f 99 "$SYNC_SUBSCRIBER" "$NUM_PUBLISHERS" &
PIDS+=($!)
echo -e "${GREEN}  PID: ${PIDS[-1]} (SCHED_FIFO priority 99)${NC}"

echo ""
echo -e "${GREEN}All nodes launched successfully!${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop all nodes${NC}"
echo ""

# Wait for all background processes
wait
