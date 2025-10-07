import argparse
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def parse_log_file(log_file_path):
    """
    Parse the PMU analyzer log file.

    Two formats are supported:
    1. PMU elapsed time log: <session_name> <counter_id> <index> <timestamp1> <timestamp2>
       - Timestamps are in microseconds
       - Latency is calculated as: timestamp1 - timestamp2

    2. ICE latency log (latency_log_ice_*): Single value per line
       - Each line contains latency in nanoseconds
       - Latency is converted to microseconds (ns / 1000)

    Returns:
        list: latencies_us (list of latencies in microseconds)
    """
    latencies_us = []
    filename = Path(log_file_path).name
    is_ice_latency_log = filename.startswith('latency_log_ice_')

    with open(log_file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue

            if is_ice_latency_log:
                # For ICE latency logs, each line contains a single latency value in nanoseconds
                if len(parts) >= 1:
                    latency_ns = int(parts[0])
                    latency_us = latency_ns / 1000.0
                    latencies_us.append(latency_us)
            else:
                # For PMU elapsed time logs, calculate difference between timestamps
                if len(parts) >= 5:
                    timestamp1 = int(parts[3])
                    timestamp2 = int(parts[4])
                    latency = timestamp1 - timestamp2
                    latencies_us.append(latency)

    return latencies_us[1:]  # filter first data


def extract_label_from_filename(filename):
    """
    Extract a readable label from the log filename.

    Expected formats:
    1. <sync_policy><max_interval_duration>_<num_publishers>
       Examples: exact50_3, approximate100_4
    2. latency_log_ice_<config>
       Examples: latency_log_ice_exact50_3

    Returns a formatted label for the plot.
    """
    name = Path(filename).stem  # Remove extension

    # Handle ICE latency logs
    if name.startswith('latency_log_ice_'):
        name = name.replace('latency_log_ice_', '')

    # Try to parse the filename
    if 'exact' in name:
        parts = name.replace('exact', '').split('_')
        if len(parts) >= 2:
            interval = parts[0]
            num_pub = parts[1]
            return f"Exact\n{interval}ms\n{num_pub}pub"
    elif 'approximate' in name:
        parts = name.replace('approximate', '').split('_')
        if len(parts) >= 2:
            interval = parts[0]
            num_pub = parts[1]
            return f"Approx\n{interval}ms\n{num_pub}pub"

    # Fallback: return the filename as-is
    return name


def plot_boxplot_comparison(log_data, output_path=None):
    """
    Create a box plot comparing latencies across different configurations.

    Args:
        log_data: Dictionary mapping labels to latency data (in microseconds)
        output_path: Optional path to save the plot
    """
    if not log_data:
        print("Error: No data to plot")
        return

    fig, ax = plt.subplots(figsize=(14, 8))

    # Prepare data for box plot
    labels = list(log_data.keys())
    data = [log_data[label] for label in labels]

    # Create box plot
    bp = ax.boxplot(data, labels=labels, patch_artist=True,
                    showmeans=True, meanline=True,
                    boxprops=dict(facecolor='lightblue', alpha=0.7),
                    medianprops=dict(color='red', linewidth=2),
                    meanprops=dict(color='green', linewidth=2, linestyle='--'),
                    whiskerprops=dict(linewidth=1.5),
                    capprops=dict(linewidth=1.5))

    ax.set_ylabel('Latency (μs)', fontsize=12)
    ax.set_xlabel('Configuration', fontsize=12)
    ax.set_title('Latency Comparison Across Configurations', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')

    # Rotate x-axis labels if needed
    plt.xticks(rotation=0, ha='center')

    # Add legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red', linewidth=2, label='Median'),
        Line2D([0], [0], color='green', linewidth=2, linestyle='--', label='Mean')
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    # Print statistics for each configuration
    print("\n=== Latency Statistics Summary ===\n")
    for label, latencies in log_data.items():
        if len(latencies) > 0:
            print(f"{label}:")
            print(f"  Samples: {len(latencies)}")
            print(f"  Min:     {np.min(latencies):10.2f} μs")
            print(f"  Max:     {np.max(latencies):10.2f} μs")
            print(f"  Mean:    {np.mean(latencies):10.2f} μs")
            print(f"  Median:  {np.median(latencies):10.2f} μs")
            print(f"  Std Dev: {np.std(latencies):10.2f} μs")
            print(f"  95th %%:  {np.percentile(latencies, 95):10.2f} μs")
            print(f"  99th %%:  {np.percentile(latencies, 99):10.2f} μs")
            print()

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize latency comparison from PMU analyzer elapsed time logs')
    parser.add_argument(
        'log_dir', type=str, nargs='?', default='pmu_analyzer_log',
        help='Path to the directory containing log files (default: pmu_analyzer_log)')
    parser.add_argument(
        '-o', '--output', type=str, default=None,
        help='Output path for the plot (e.g., latency_comparison.png). If not specified, displays interactively.')
    parser.add_argument('--pattern', type=str, default='*',
                        help='File pattern to match (default: * for all files in directory)')

    args = parser.parse_args()

    log_dir = Path(args.log_dir)

    # Check if log_dir exists
    if not log_dir.exists():
        print(f"Error: Log directory not found: {log_dir}")
        return 1

    # If log_dir is a file, treat it as a single log file for backward compatibility
    if log_dir.is_file():
        print(f"Reading single log file: {log_dir}")
        latencies = parse_log_file(log_dir)
        if len(latencies) == 0:
            print("Error: No data found in log file")
            return 1

        label = extract_label_from_filename(log_dir.name)
        log_data = {label: latencies}
        print(f"Loaded {len(latencies)} samples from {log_dir.name}")
    else:
        # Read all log files in the directory
        print(f"Reading log files from directory: {log_dir}")
        log_files = sorted(log_dir.glob(args.pattern))

        if not log_files:
            print(f"Error: No log files found in {log_dir} matching pattern '{args.pattern}'")
            return 1

        log_data = {}
        for log_file in log_files:
            if log_file.is_file():
                print(f"  Processing: {log_file.name}")
                latencies = parse_log_file(log_file)
                if len(latencies) > 0:
                    label = extract_label_from_filename(log_file.name)
                    log_data[label] = latencies
                    print(f"    Loaded {len(latencies)} samples")
                else:
                    print(f"    Warning: No data found in {log_file.name}")

        if not log_data:
            print("Error: No valid data found in any log files")
            return 1

        print(f"\nTotal configurations: {len(log_data)}")

    # Create box plot comparison
    print("\nGenerating box plot comparison...")
    plot_boxplot_comparison(log_data, args.output)

    return 0


if __name__ == '__main__':
    exit(main())
