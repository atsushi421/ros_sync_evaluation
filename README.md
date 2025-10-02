# ROS 2 Sync Evaluation

This package provides a parametric evaluation framework for `message_filters::sync_policies::ExactTime` to synchronize messages from multiple publishers (1-4) with coordinated startup timing. All nodes run in a single process using ROS 2 component containers for minimal inter-process communication overhead.

## Package Structure

- `source_publisher`: Publishes continuous start signals (10Hz) to synchronize all publishers
- `subscribe_publisher`: Parametric publisher that reacts to start signals with random pseudo processing
- `sync_subscriber`: Parametric subscriber supporting 1-4 synchronized topics using ExactTime policy
- `custom_msg`: Custom message package defining HeaderExtraStamp message

## Message Flow

- `source_publisher` → `start_topic` (custom_msg::msg::HeaderExtraStamp) → all `subscribe_publisher` instances
- Each `subscribe_publisher` → `topic1`, `topic2`, `topic3`, or `topic4` → `sync_subscriber`
- `sync_subscriber` uses ExactTime policy to match messages with identical timestamps

## Building

```bash
cd /home/atsushi/ros_sync_evaluation
colcon build
source install/setup.bash
```

## Running

The recommended way to run the evaluation uses a single component container process with SCHED_FIFO priority 99:

```bash
# Launch all nodes in single process with RT priority
sudo scripts/run_with_rt_priority.sh <num_publishers>
```

Example for 2 publishers:
```bash
sudo scripts/run_with_rt_priority.sh 2
```

## Visualization

Analyze PMU analyzer logs:

```bash
python3 scripts/visualize_latency.py pmu_analyzer_log/elapsed_time_log_<PID>_0
```

This generates:
- Latency time series plot
- Latency histogram
- Statistical summary (min/max/mean/median/std/percentiles)
