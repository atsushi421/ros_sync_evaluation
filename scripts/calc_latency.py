#!/usr/bin/env python3

import argparse
import glob
from pathlib import Path
from typing import List, Dict


SYNC_SESSION_NAME = "sync_subscriber_subscribed"


def calculate_latencies(log_dir: Path) -> List[int]:
    # Read log files
    log_pattern = str(log_dir / "elapsed_time_log_*")
    files = sorted(glob.glob(log_pattern))

    if not files:
        print(f"Warning: No log files found matching pattern: {log_pattern}")
        return []

    print(f"Found {len(files)} log file(s)")

    # Parse all log entries
    sync_entries = []
    publish_times_by_id: Dict[int, List[int]] = {}

    for file_path in files:
        print(f"  Processing: {Path(file_path).name}")
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                tokens = line.strip().split()
                if not tokens:
                    continue

                session_name = tokens[0]
                message_id = int(tokens[2])
                timestamp = int(tokens[3])

                if session_name == SYNC_SESSION_NAME:
                    sync_entries.append((message_id, timestamp))
                else:
                    publish_times_by_id.setdefault(message_id, []).append(timestamp)

    print(f"Sync entries: {len(sync_entries)}")
    print(f"Publish entry groups: {len(publish_times_by_id)}")

    # Calculate latencies
    latencies = []
    for message_id, sync_time in sync_entries:
        publish_times = publish_times_by_id.get(message_id)
        if publish_times:
            latency = sync_time - max(publish_times)
            latencies.append(latency)

    print(f"Calculated {len(latencies)} latencies")
    return latencies


def cleanup_log_files(log_dir: Path) -> None:
    log_pattern = str(log_dir / "elapsed_time_log_*")
    log_files = glob.glob(log_pattern)

    if not log_files:
        return

    print(f"\nRemoving {len(log_files)} log file(s)...")
    for log_file in log_files:
        try:
            Path(log_file).unlink()
            print(f"  Removed: {Path(log_file).name}")
        except Exception as e:
            print(f"  Warning: Failed to remove {Path(log_file).name}: {e}")

    print("Log files cleanup complete")


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Calculate latencies from PMU analyzer elapsed time logs')
    parser.add_argument('log_dir', type=str, help='Base directory containing log files')
    parser.add_argument('num_publishers', type=int, help='Number of publishers')
    parser.add_argument(
        'sync_policy', type=str, choices=['exact', 'approximate'],
        help='Synchronization policy')
    parser.add_argument('max_interval_duration', type=str,
                        help='Max interval duration in milliseconds')

    args = parser.parse_args()

    log_dir = Path(args.log_dir)
    if not log_dir.exists():
        print(f"Error: Log directory not found: {log_dir}")
        return 1

    print(f"Calculating latencies from {log_dir}")
    print(
        f"Configuration: {args.sync_policy}, {args.max_interval_duration}ms, {args.num_publishers} publishers")

    latencies = calculate_latencies(log_dir)

    if not latencies:
        print("Warning: No latencies calculated")
        return 1

    # Write output
    output_dir = log_dir / str(args.num_publishers)
    output_dir.mkdir(parents=True, exist_ok=True)

    output_filename = f"{args.sync_policy}{args.max_interval_duration}_latencies_us.txt"
    output_path = output_dir / output_filename

    with open(output_path, "w", encoding="utf-8") as f:
        for latency in latencies:
            f.write(f"{latency}\n")

    print(f"\nOutput written to: {output_path}")
    print(f"Total latencies: {len(latencies)}")

    cleanup_log_files(log_dir)

    return 0


if __name__ == "__main__":
    exit(main())
