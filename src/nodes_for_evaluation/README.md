# ROS 2 Sync Evaluation

This package provides a parametric evaluation framework for `message_filters::sync_policies::ExactTime` to synchronize messages from multiple publishers (1-4) with coordinated startup timing.

## Package Structure

- `source_publisher`: Publishes continuous start signals (10Hz) to synchronize all publishers
- `subscribe_publisher`: Parametric publisher that reacts to start signals with random pseudo processing
- `sync_subscriber`: Parametric subscriber supporting 1-4 synchronized topics using ExactTime policy
- `custom_msg`: Custom message package defining HeaderExtraStamp message

## Building

The workspace should be structured with both packages accessible:

```bash
cd /home/atsushi/ros_sync_ws
colcon build
source install/setup.bash
```

## Running

### Recommended: Launch all nodes with real-time priority

For optimal real-time performance, use the wrapper script that launches all nodes with the subscriber running at SCHED_FIFO priority 99:

```bash
# Launch all nodes (source_publisher, N subscribe_publishers, sync_subscriber with RT priority)
sudo scripts/run_with_rt_priority.sh <num_publishers> install/nodes_for_evaluation
```

Example for 3 publishers:
```bash
sudo scripts/run_with_rt_priority.sh 3 install/nodes_for_evaluation
```

### Alternative: Launch file (without real-time priority)

```bash
ros2 launch nodes_for_evaluation evaluation.launch.py num_publishers:=3
```

**Note:** The launch file automatically sets `PMU_ANALYZER_CONFIG_FILE` for all nodes but does not provide real-time scheduling.

### Run nodes individually

**Important:** Set the PMU analyzer config file path before running:
```bash
export PMU_ANALYZER_CONFIG_FILE=/home/atsushi/ros_sync_evaluation/config/pmu_config.yaml
```

Terminal 1 (Start signal):
```bash
ros2 run nodes_for_evaluation source_publisher
```

Terminal 2-N (Publishers):
```bash
ros2 run nodes_for_evaluation subscribe_publisher --ros-args -p topic_id:=1
ros2 run nodes_for_evaluation subscribe_publisher --ros-args -p topic_id:=2
ros2 run nodes_for_evaluation subscribe_publisher --ros-args -p topic_id:=3
```

Terminal N+1 (Subscriber):
```bash
# Without RT priority
ros2 run nodes_for_evaluation sync_subscriber 3
```

## Expected Behavior

1. `source_publisher` waits 2 seconds, then begins broadcasting start signals continuously at 10Hz
2. Each `subscribe_publisher` waits for start signals on `start_topic`
3. Upon receiving a start signal, each publisher:
   - Performs random pseudo processing (10000-10000000 iterations of sin/cos calculations)
   - Publishes the received message directly (preserving timestamp, frame_id, and point data)
4. All publishers are synchronized by the same start signal stream
5. `sync_subscriber` uses ExactTime policy to synchronize messages from all publishers
6. PMU analyzer captures performance metrics throughout execution for all nodes

## Configuration

### Publisher Parameters
- `topic_id`: Topic number (1-4)

### Launch Parameters
- `num_publishers`: Number of publishers to launch (1-4, default: 2)

### Timing Behavior
- Start signals are published at 10Hz (100ms interval) after 2-second initial delay
- Each publisher performs random pseudo processing (10000-10000000 iterations) upon receiving start signal
- Publishers forward the received message directly, preserving timestamp, frame_id, and point data
- This ensures all publishers are synchronized to the same clock source

### Message Flow
- `source_publisher` → `start_topic` (custom_msg::msg::HeaderExtraStamp) → all `subscribe_publisher` instances
- Each `subscribe_publisher` → `topic1`, `topic2`, `topic3`, or `topic4` → `sync_subscriber`
- `sync_subscriber` uses ExactTime policy to match messages with identical timestamps
- HeaderExtraStamp messages contain both header.stamp and extra_stamp fields
