import argparse
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from collections import defaultdict


def parse_log_file(log_file_path):
    """
    Parse latency log files.

    Supports two formats:
    1. Latency files (*_latencies_us.txt): Single latency value (in μs) per line
    2. ICE latency logs (latency_log_ice_*): Single latency value (in ns) per line

    Returns:
        list: latencies_us (list of latencies in microseconds)
    """
    latencies_us = []
    filename = Path(log_file_path).name
    is_ice_latency_log = filename.startswith('latency_log_ice_')

    with open(log_file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            try:
                if is_ice_latency_log:
                    # ICE logs: latency in nanoseconds
                    latency_ns = int(line)
                    latencies_us.append(latency_ns / 1000.0)
                else:
                    # Standard latency files: latency in microseconds
                    latencies_us.append(int(line))
            except ValueError:
                continue

    # Filter first data point
    return latencies_us[1:] if len(latencies_us) > 1 else latencies_us


def extract_label_from_filename(filename, parent_dir_name=None):
    """
    Extract a readable label from the log filename.

    Returns:
        tuple: (num_publishers, label_text)
    """
    name = Path(filename).stem  # Remove extension

    # Handle ICE latency logs
    if name.startswith('latency_log_ice_'):
        num_pub = name.replace('latency_log_ice_', '')
        return (num_pub, "Awkernel")

    # Extract num_publishers from parent directory name
    num_pub = parent_dir_name if parent_dir_name and parent_dir_name.isdigit() else "?"

    # Remove '_latencies_us' suffix
    if name.endswith('_latencies_us'):
        name = name[:-13]

    # Parse sync policy and interval
    if name.startswith('exact'):
        interval = name.replace('exact', '')
        return (num_pub, f"Exact")
    elif name.startswith('approximate'):
        interval = name.replace('approximate', '')
        return (num_pub, f"Approx {interval}ms")

    # Fallback
    return (num_pub, name)


def plot_boxplot_comparison(log_data_by_num_pub, output_dir=None):
    """
    Create separate box plots for each num_publishers group.

    Args:
        log_data_by_num_pub: Dictionary mapping num_publishers to {label: latencies}
        output_dir: Optional directory to save plots (if None, display interactively)
    """
    if not log_data_by_num_pub:
        print("Error: No data to plot")
        return

    for num_pub, log_data in sorted(log_data_by_num_pub.items()):
        print(f"\n=== {num_pub} Publishers ===")

        if not log_data:
            continue

        _, ax = plt.subplots(figsize=(12, 8))

        # Prepare data for box plot
        labels = list(log_data.keys())
        data = [log_data[label] for label in labels]

        # Create box plot
        ax.boxplot(
            data,
            labels=labels,
            patch_artist=True,
            showmeans=True,
            meanline=True,
            boxprops=dict(facecolor="#3D649B", edgecolor="#34495E", linewidth=1.5, alpha=0.85),
            medianprops=dict(color='none'),
            meanprops=dict(color="#34495E", linewidth=2, linestyle='-'),
            whiskerprops=dict(color='#34495E', linewidth=1.8, linestyle='-'),
            capprops=dict(color='#34495E', linewidth=1.8)
        )

        ax.set_ylabel('Latency (μs)', fontsize=12)
        ax.set_yscale('log')
        ax.set_title(f'{num_pub} Publishers', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='y')
        plt.xticks(rotation=0, ha='center')

        # Print statistics
        print("\nStatistics:")
        for label, latencies in log_data.items():
            if len(latencies) > 0:
                print(f"\n{label}:")
                print(f"  Samples: {len(latencies)}")
                print(f"  Min:     {np.min(latencies):10.2f} μs")
                print(f"  Max:     {np.max(latencies):10.2f} μs")
                print(f"  Mean:    {np.mean(latencies):10.2f} μs")
                print(f"  Median:  {np.median(latencies):10.2f} μs")
                print(f"  Std Dev: {np.std(latencies):10.2f} μs")
                print(f"  95th %%:  {np.percentile(latencies, 95):10.2f} μs")
                print(f"  99th %%:  {np.percentile(latencies, 99):10.2f} μs")

        plt.tight_layout()

        # Save or display
        if output_dir:
            output_path = Path(output_dir) / f"latency_comparison_{num_pub}pub.png"
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"\nPlot saved to: {output_path}")
            plt.close()
        else:
            plt.show()


def collect_log_files(log_dir, pattern):
    """
    Recursively collect log files from directory structure.

    Searches for files matching pattern in log_dir and subdirectories.
    """
    log_files = []

    # Search in root directory
    log_files.extend(log_dir.glob(pattern))

    # Search in numbered subdirectories (1-8)
    for i in range(1, 9):
        subdir = log_dir / str(i)
        if subdir.exists() and subdir.is_dir():
            log_files.extend(subdir.glob(pattern))

    return sorted(log_files)


def main():
    parser = argparse.ArgumentParser(
        description='Visualize latency comparison from PMU analyzer latency logs')
    parser.add_argument(
        'log_dir', type=str, nargs='?', default='pmu_analyzer_log',
        help='Path to the directory containing log files (default: pmu_analyzer_log)')
    parser.add_argument(
        '-o', '--output', type=str, default=None,
        help='Output directory for plots (e.g., plots/)')
    parser.add_argument(
        '--pattern', type=str, default='*',
        help='File pattern to match (default: *')

    args = parser.parse_args()

    log_dir = Path(args.log_dir)

    # Check if log_dir exists
    if not log_dir.exists():
        print(f"Error: Log directory not found: {log_dir}")
        return 1

    # If log_dir is a file, treat it as a single log file
    if log_dir.is_file():
        print(f"Reading single log file: {log_dir}")
        latencies = parse_log_file(log_dir)
        if len(latencies) == 0:
            print("Error: No data found in log file")
            return 1

        parent_dir_name = log_dir.parent.name
        num_pub, label = extract_label_from_filename(log_dir.name, parent_dir_name)
        log_data_by_num_pub = {num_pub: {label: latencies}}
        print(f"Loaded {len(latencies)} samples from {log_dir.name}")
    else:
        # Read all log files in the directory and subdirectories
        print(f"Reading log files from directory: {log_dir}")
        log_files = collect_log_files(log_dir, args.pattern)

        if not log_files:
            print(f"Error: No log files found in {log_dir} matching pattern '{args.pattern}'")
            return 1

        # Group by num_publishers
        log_data_by_num_pub = defaultdict(dict)

        for log_file in log_files:
            if log_file.is_file():
                print(f"  Processing: {log_file.relative_to(log_dir)}")
                latencies = parse_log_file(log_file)
                if len(latencies) > 0:
                    parent_dir_name = log_file.parent.name
                    num_pub, label = extract_label_from_filename(log_file.name, parent_dir_name)
                    log_data_by_num_pub[num_pub][label] = latencies
                    print(f"    Loaded {len(latencies)} samples")
                else:
                    print(f"    Warning: No data found in {log_file.name}")

        if not log_data_by_num_pub:
            print("Error: No valid data found in any log files")
            return 1

        print(f"\nTotal publisher groups: {len(log_data_by_num_pub)}")
        print(f"Total configurations: {sum(len(v) for v in log_data_by_num_pub.values())}")

    # Create output directory if specified
    output_dir = None
    if args.output:
        output_dir = Path(args.output)
        output_dir.mkdir(parents=True, exist_ok=True)

    # Create box plot comparisons
    print("\nGenerating box plot comparisons...")
    plot_boxplot_comparison(log_data_by_num_pub, output_dir)

    return 0


if __name__ == '__main__':
    exit(main())
