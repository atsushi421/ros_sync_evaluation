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
	echo "Usage: sudo $0 [workspace_root] [num_publishers]> [sync_policy] [max_interval_duration]"
	exit 1
fi

# Parse arguments
if [ $# -lt 1 ]; then
	echo -e "${RED}Error: Missing required arguments${NC}"
	echo ""
	echo "Usage: sudo $0 [workspace_root] [num_publishers]> [sync_policy] [max_interval_duration]"
	echo ""
	echo "Arguments:"
	echo "  workspace_root: Path to workspace (default: \$HOME/ros_sync_evaluation)"
	echo "  num_publishers: Number of publishers to launch (1-8)"
	echo "  sync_policy: Synchronization policy - 'exact' or 'approximate' (default: exact)"
	echo "  max_interval_duration: Max interval for approximate sync in milliseconds (default: 50)"
	echo ""
	echo "Examples:"
	echo "  sudo $0 /home/atsushi/ros_sync_evaluation 3"
	echo "  sudo $0 /home/atsushi/ros_sync_evaluation 3 exact"
	echo "  sudo $0 /home/atsushi/ros_sync_evaluation 3 approximate 50"
	echo "  sudo $0 /home/atsushi/ros_sync_evaluation 3 approximate 100"
	exit 1
fi

WORKSPACE_ROOT="$1"
NUM_PUBLISHERS="$2"
SYNC_POLICY="${3:-exact}"
MAX_INTERVAL_DURATION="${4:-50}"

# Validate number of publishers
if ! [[ "$NUM_PUBLISHERS" =~ ^[1-8]$ ]]; then
	echo -e "${RED}Error: num_publishers must be 1, 2, 3, 4, 5, 6, 7, or 8${NC}"
	exit 1
fi

# Validate sync policy
if [[ "$SYNC_POLICY" != "exact" && "$SYNC_POLICY" != "approximate" ]]; then
	echo -e "${RED}Error: sync_policy must be 'exact' or 'approximate'${NC}"
	exit 1
fi

# Validate workspace root
if [ ! -d "$WORKSPACE_ROOT" ]; then
	echo -e "${RED}Error: Workspace root directory not found: $WORKSPACE_ROOT${NC}"
	exit 1
fi

# Source ROS 2 environment
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
	source "$WORKSPACE_ROOT/install/setup.bash"
	echo -e "${GREEN}Sourced ROS 2 workspace: $WORKSPACE_ROOT/install/setup.bash${NC}"
else
	echo -e "${YELLOW}Warning: setup.bash not found at $WORKSPACE_ROOT/install/setup.bash${NC}"
fi

# Set PMU analyzer config file if not already set
if [ -z "$PMU_ANALYZER_CONFIG_FILE" ]; then
	CONFIG_FILE="$WORKSPACE_ROOT/config/pmu_config.yaml"

	if [ -f "$CONFIG_FILE" ]; then
		export PMU_ANALYZER_CONFIG_FILE="$CONFIG_FILE"
		echo -e "${GREEN}Set PMU_ANALYZER_CONFIG_FILE to: $CONFIG_FILE${NC}"
	else
		echo -e "${YELLOW}Warning: PMU config file not found at $CONFIG_FILE${NC}"
		echo -e "${YELLOW}Set PMU_ANALYZER_CONFIG_FILE environment variable if needed${NC}"
	fi
fi

# Resolve PMU log directory from config (supports absolute and relative paths)
LOG_DIR="$WORKSPACE_ROOT/pmu_analyzer_log"
if [ -n "$PMU_ANALYZER_CONFIG_FILE" ] && [ -f "$PMU_ANALYZER_CONFIG_FILE" ]; then
	CONFIG_LOG_PATH=$(python3 - "$PMU_ANALYZER_CONFIG_FILE" <<'PY'
from pathlib import Path
import sys

cfg = Path(sys.argv[1])
for line in cfg.read_text(encoding="utf-8", errors="ignore").splitlines():
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        continue
    if stripped.startswith("log_path:"):
        value = stripped.split(":", 1)[1].strip().strip('"\'')
        print(value)
        break
PY
)

	if [ -n "$CONFIG_LOG_PATH" ]; then
		if [[ "$CONFIG_LOG_PATH" = /* ]]; then
			LOG_DIR="$CONFIG_LOG_PATH"
		else
			LOG_DIR="$WORKSPACE_ROOT/$CONFIG_LOG_PATH"
		fi
	fi
fi

# Ensure log directory exists before node startup
mkdir -p "$LOG_DIR"

# Display configuration
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ROS 2 Sync Evaluation Launcher (Component Container)${NC}"
echo -e "${GREEN}========================================${NC}"
echo "  Workspace Root: $WORKSPACE_ROOT"
echo "  Number of Publishers: $NUM_PUBLISHERS"
echo "  Sync Policy: $SYNC_POLICY"
echo "  Max Interval Duration: ${MAX_INTERVAL_DURATION}ms"
echo "  Mode: Single Process (All Nodes in Component Container)"
echo "  RT Priority: SCHED_FIFO 99"
echo "  PMU Config: ${PMU_ANALYZER_CONFIG_FILE:-<not set>}"
echo "  PMU Log Dir: $LOG_DIR"
echo ""

# Cleanup function to kill all background processes
cleanup() {
	echo ""
	echo -e "${YELLOW}Shutting down all nodes...${NC}"
	if [ -n "$CONTAINER_PID" ] && kill -0 "$CONTAINER_PID" 2>/dev/null; then
		kill "$CONTAINER_PID" 2>/dev/null || true
	fi
	wait
	echo -e "${GREEN}All nodes stopped${NC}"

	sleep 2 # Wait for log files to be written

	python3 "$WORKSPACE_ROOT/scripts/calc_latency.py" "$LOG_DIR" $NUM_PUBLISHERS $SYNC_POLICY $MAX_INTERVAL_DURATION

	exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Launch file path
LAUNCH_FILE="$WORKSPACE_ROOT/launch/component_container.launch.py"

if [ ! -f "$LAUNCH_FILE" ]; then
	echo -e "${RED}Error: Launch file not found at $LAUNCH_FILE${NC}"
	exit 1
fi

# Launch component container with all nodes (including sync_subscriber) in single process
echo -e "${BLUE}Launching component container (all nodes in single process)...${NC}"
ros2 launch "$LAUNCH_FILE" num_publishers:="$NUM_PUBLISHERS" sync_policy:="$SYNC_POLICY" max_interval_duration:="$MAX_INTERVAL_DURATION" &
CONTAINER_PID=$!
echo -e "${GREEN}  Container PID: $CONTAINER_PID${NC}"

# Wait a moment for the container process to start
sleep 2

# Set RT priority (SCHED_FIFO 99) for the component container process
if [ -n "$CONTAINER_PID" ] && kill -0 "$CONTAINER_PID" 2>/dev/null; then
	echo -e "${BLUE}Setting SCHED_FIFO priority 99 for container process...${NC}"
	if chrt -f -p 99 "$CONTAINER_PID"; then
		echo -e "${GREEN}  Successfully set RT priority SCHED_FIFO 99 for PID: $CONTAINER_PID${NC}"
	else
		echo -e "${YELLOW}  Warning: Failed to set RT priority${NC}"
	fi
else
	echo -e "${YELLOW}  Warning: Container process not found, skipping RT priority setting${NC}"
fi

echo ""
echo -e "${GREEN}All nodes launched successfully in single process!${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop all nodes${NC}"
echo ""

# Wait for all background processes
wait
