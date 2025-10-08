# ROS 2 Sync Evaluation

This package provides a parametric evaluation framework for ROS 2 message synchronization policies (`message_filters::sync_policies::ExactTime` and `ApproximateTime`) to synchronize messages from multiple publishers (1-8) with coordinated startup timing. All nodes run in a single process using ROS 2 component containers for minimal inter-process communication overhead.

## Package Structure

- `source_publisher`: Publishes continuous start signals (10Hz) to coordinate all publishers
- `subscribe_publisher`: Parametric publisher that reacts to start signals with random pseudo processing (1000-50000 μs)
  - **ExactTime mode**: Updates timestamps to ensure identical values across publishers
  - **ApproximateTime mode**: Preserves original timestamps, allowing natural time variations
- `sync_subscriber`: Parametric subscriber supporting 1-8 synchronized topics using ExactTime or ApproximateTime policy
- `custom_msg`: Custom message package defining HeaderExtraStamp message

## Message Flow

1. `source_publisher` → `start_topic` (custom_msg::msg::HeaderExtraStamp) → all `subscribe_publisher` instances
2. Each `subscribe_publisher` (1-8) → `topic1`...`topic8` → `sync_subscriber`
3. `sync_subscriber` uses ExactTime or ApproximateTime policy to match messages

## Timestamp Behavior

The framework implements policy-aware timestamp management:

- **ExactTime Policy**:
  - Publishers update `msg->header.stamp` to `this->now()` after processing
  - Ensures all messages have identical timestamps for exact synchronization

- **ApproximateTime Policy**:
  - Publishers preserve original timestamps from `source_publisher`
  - Allows natural timestamp variations due to processing delays
  - Sync subscriber matches messages within configured time window

## Building

```bash
cd ros_sync_evaluation
colcon build
source install/setup.bash
```

## Running

The recommended way to run the evaluation uses a single component container process with SCHED_FIFO priority 99:

```bash
sudo scripts/run_with_rt_priority.sh <workspace_root> <num_publishers> [sync_policy] [max_interval_duration]
```

**Arguments:**
- `workspace_root`: Path to workspace (required)
  - Example: `/home/user/ros_sync_evaluation`
- `num_publishers`: Number of publishers to launch (required, 1-8)
- `sync_policy`: Synchronization policy (optional)
  - Values: `exact` or `approximate`
  - Default: `exact`
- `max_interval_duration`: Max interval for approximate sync in milliseconds (optional)
  - Default: `50`

**Examples:**

```bash
# 3 publishers with exact time sync
sudo scripts/run_with_rt_priority.sh /home/user/ros_sync_evaluation 3 exact

# 4 publishers with approximate sync, 100ms window
sudo scripts/run_with_rt_priority.sh /home/user/ros_sync_evaluation 4 approximate 100

# Press Ctrl+C to stop evaluation
# Latencies are automatically calculated and logs are cleaned up on exit
```

## Visualization

Visualize and compare latencies across different configurations with separate plots per publisher count:

```bash
python3 scripts/visualize_latency.py pmu_analyzer_log -o plots/
# Output: plots/latency_comparison_3pub.png, plots/latency_comparison_4pub.png, etc.
```
