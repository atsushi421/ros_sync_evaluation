import argparse
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def parse_log_file(log_file_path):
    """
    Parse the PMU analyzer log file.

    Format: <session_name> <counter_id> <index> <timestamp1> <timestamp2>
    Note: Timestamps are in microseconds

    Returns:
        tuple: (indices, latencies_us, latencies_ms)
    """
    indices = []
    latencies_us = []

    with open(log_file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                index = int(parts[2])
                timestamp1 = int(parts[3])
                timestamp2 = int(parts[4])
                latency = timestamp1 - timestamp2

                indices.append(index)
                latencies_us.append(latency)

    # Convert to milliseconds
    latencies_ms = [lat / 1000.0 for lat in latencies_us]

    return indices, latencies_us, latencies_ms


def plot_latency(indices, latencies_us, log_file_path, output_path=None):
    """
    Plot latency over time.

    Args:
        indices: List of message indices
        latencies_us: List of latencies in microseconds
        log_file_path: Path to the log file (for title)
        output_path: Optional path to save the plot
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Calculate statistics
    mean_latency = np.mean(latencies_us)
    median_latency = np.median(latencies_us)
    min_latency = np.min(latencies_us)
    max_latency = np.max(latencies_us)
    std_latency = np.std(latencies_us)

    # Plot 1: Latency over time
    ax1.plot(indices, latencies_us, marker='o', markersize=3, linestyle='-', linewidth=1)
    ax1.axhline(y=mean_latency, color='r', linestyle='--', label=f'Mean: {mean_latency:.2f} μs')
    ax1.axhline(y=median_latency, color='g', linestyle='--',
                label=f'Median: {median_latency:.2f} μs')
    ax1.set_xlabel('Message Index')
    ax1.set_ylabel('Latency (μs)')
    ax1.set_title(f'Message Latency Over Time\n{Path(log_file_path).name}')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Plot 2: Histogram
    ax2.hist(latencies_us, bins=50, edgecolor='black', alpha=0.7)
    ax2.axvline(x=mean_latency, color='r', linestyle='--', label=f'Mean: {mean_latency:.2f} μs')
    ax2.axvline(x=median_latency, color='g', linestyle='--',
                label=f'Median: {median_latency:.2f} μs')
    ax2.set_xlabel('Latency (μs)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Latency Distribution')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    # Add statistics text
    stats_text = f'Statistics:\n'
    stats_text += f'Min:    {min_latency:.2f} μs\n'
    stats_text += f'Max:    {max_latency:.2f} μs\n'
    stats_text += f'Mean:   {mean_latency:.2f} μs\n'
    stats_text += f'Median: {median_latency:.2f} μs\n'
    stats_text += f'Std:    {std_latency:.2f} μs\n'
    stats_text += f'Samples: {len(latencies_us)}'

    fig.text(0.02, 0.02, stats_text, fontsize=10, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout(rect=[0, 0.12, 1, 1])

    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()


def print_statistics(indices, latencies_us, latencies_ms):
    """Print latency statistics to console."""
    print("\n=== Latency Statistics ===")
    print(f"Total samples:     {len(latencies_us)}")
    print(f"\nLatency (microseconds):")
    print(f"  Min:     {np.min(latencies_us):12.0f} μs")
    print(f"  Max:     {np.max(latencies_us):12.0f} μs")
    print(f"  Mean:    {np.mean(latencies_us):12.2f} μs")
    print(f"  Median:  {np.median(latencies_us):12.2f} μs")
    print(f"  Std Dev: {np.std(latencies_us):12.2f} μs")
    print(f"\nLatency (milliseconds):")
    print(f"  Min:     {np.min(latencies_ms):12.2f} ms")
    print(f"  Max:     {np.max(latencies_ms):12.2f} ms")
    print(f"  Mean:    {np.mean(latencies_ms):12.2f} ms")
    print(f"  Median:  {np.median(latencies_ms):12.2f} ms")
    print(f"  Std Dev: {np.std(latencies_ms):12.2f} ms")

    # Percentiles
    p50 = np.percentile(latencies_us, 50)
    p95 = np.percentile(latencies_us, 95)
    p99 = np.percentile(latencies_us, 99)
    p999 = np.percentile(latencies_us, 99.9)

    print(f"\nPercentiles (microseconds):")
    print(f"  50th:    {p50:12.2f} μs")
    print(f"  95th:    {p95:12.2f} μs")
    print(f"  99th:    {p99:12.2f} μs")
    print(f"  99.9th:  {p999:12.2f} μs")


def main():
    parser = argparse.ArgumentParser(
        description='Visualize latency from PMU analyzer elapsed time logs')
    parser.add_argument('log_file', type=str,
                        help='Path to the PMU analyzer log file')
    parser.add_argument(
        '-o', '--output', type=str, default=None,
        help='Output path for the plot (e.g., latency.png). If not specified, displays interactively.')
    parser.add_argument('--no-plot', action='store_true',
                        help='Only print statistics without plotting')

    args = parser.parse_args()

    log_file = Path(args.log_file)
    if not log_file.exists():
        print(f"Error: Log file not found: {log_file}")
        return 1

    print(f"Reading log file: {log_file}")
    indices, latencies_us, latencies_ms = parse_log_file(log_file)

    if len(indices) == 0:
        print("Error: No data found in log file")
        return 1

    print(f"Loaded {len(indices)} samples")

    # Print statistics
    print_statistics(indices, latencies_us, latencies_ms)

    # Plot if requested
    if not args.no_plot:
        print("\nGenerating plot...")
        plot_latency(indices, latencies_us, log_file, args.output)

    return 0


if __name__ == '__main__':
    exit(main())
