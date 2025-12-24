#!/usr/bin/env python3
"""Plot EKF performance with ground truth comparison and RMSE analysis."""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_trajectory(df, output_dir):
    """Plot 2D trajectory: Ground truth vs EKF vs Raw GPS."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Raw GPS (noisy)
    ax.plot(df['gps_x'], df['gps_y'], 'r.', alpha=0.2, markersize=2, 
            label='Raw GPS', zorder=1)
    
    # Ground truth (from odom)
    ax.plot(df['gt_x'], df['gt_y'], 'g-', linewidth=2, 
            label='Ground Truth', zorder=2)
    
    # EKF estimate
    ax.plot(df['ekf_x'], df['ekf_y'], 'b--', linewidth=1.5, 
            label='EKF Estimate', zorder=3)

    # Start and end markers
    ax.plot(df['gt_x'].iloc[0], df['gt_y'].iloc[0], 'ko', 
            markersize=12, label='Start', zorder=4)
    ax.plot(df['gt_x'].iloc[-1], df['gt_y'].iloc[-1], 'k^', 
            markersize=12, label='End', zorder=4)

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Robot Trajectory Comparison', fontsize=14)
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'trajectory.png'), dpi=150)
    plt.close()
    print("  ✓ trajectory.png")


def plot_error_vs_time(df, output_dir):
    """Plot estimation error over time for x, y, yaw."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    err_x = df['ekf_x'] - df['gt_x']
    err_y = df['ekf_y'] - df['gt_y']
    err_theta = np.degrees(df['ekf_theta'] - df['gt_theta'])
    
    # Wrap angle error to [-180, 180]
    err_theta = np.where(err_theta > 180, err_theta - 360, err_theta)
    err_theta = np.where(err_theta < -180, err_theta + 360, err_theta)

    # X error
    axes[0].plot(df['time'], err_x, 'b-', linewidth=0.8)
    axes[0].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[0].set_ylabel('X Error (m)', fontsize=11)
    axes[0].set_title('Estimation Error vs Time', fontsize=14)
    axes[0].grid(True, alpha=0.3)

    # Y error
    axes[1].plot(df['time'], err_y, 'g-', linewidth=0.8)
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Y Error (m)', fontsize=11)
    axes[1].grid(True, alpha=0.3)

    # Yaw error
    axes[2].plot(df['time'], err_theta, 'r-', linewidth=0.8)
    axes[2].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Yaw Error (deg)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'error_vs_time.png'), dpi=150)
    plt.close()
    print("  ✓ error_vs_time.png")


def plot_covariance_consistency(df, output_dir):
    """Plot error with ±2σ bounds to show covariance consistency."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    err_x = df['ekf_x'] - df['gt_x']
    err_y = df['ekf_y'] - df['gt_y']
    err_theta = df['ekf_theta'] - df['gt_theta']
    
    # Wrap angle error
    err_theta = np.arctan2(np.sin(err_theta), np.cos(err_theta))

    sigma_x = np.sqrt(df['ekf_cov_x'])
    sigma_y = np.sqrt(df['ekf_cov_y'])
    sigma_theta = np.sqrt(df['ekf_cov_theta'])

    # X: error with ±2σ bounds
    axes[0].fill_between(df['time'], -2*sigma_x, 2*sigma_x, 
                         alpha=0.3, color='blue', label='±2σ')
    axes[0].plot(df['time'], err_x, 'b-', linewidth=0.8, label='Error')
    axes[0].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[0].set_ylabel('X Error (m)', fontsize=11)
    axes[0].set_title('Covariance Consistency (Error with ±2σ Bounds)', fontsize=14)
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)

    # Y: error with ±2σ bounds
    axes[1].fill_between(df['time'], -2*sigma_y, 2*sigma_y, 
                         alpha=0.3, color='green', label='±2σ')
    axes[1].plot(df['time'], err_y, 'g-', linewidth=0.8, label='Error')
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Y Error (m)', fontsize=11)
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # Yaw: error with ±2σ bounds
    axes[2].fill_between(df['time'], np.degrees(-2*sigma_theta), 
                         np.degrees(2*sigma_theta), 
                         alpha=0.3, color='red', label='±2σ')
    axes[2].plot(df['time'], np.degrees(err_theta), 'r-', linewidth=0.8, 
                 label='Error')
    axes[2].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Yaw Error (deg)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'covariance_consistency.png'), dpi=150)
    plt.close()
    print("  ✓ covariance_consistency.png")


def compute_rmse(df):
    """Compute RMSE for EKF vs Ground Truth and GPS vs Ground Truth."""
    # EKF errors
    ekf_err_x = df['ekf_x'] - df['gt_x']
    ekf_err_y = df['ekf_y'] - df['gt_y']
    ekf_err_theta = df['ekf_theta'] - df['gt_theta']
    ekf_err_theta = np.arctan2(np.sin(ekf_err_theta), np.cos(ekf_err_theta))

    # GPS errors (raw sensor)
    gps_err_x = df['gps_x'] - df['gt_x']
    gps_err_y = df['gps_y'] - df['gt_y']

    # IMU errors (raw sensor)
    imu_err_theta = df['imu_theta'] - df['gt_theta']
    imu_err_theta = np.arctan2(np.sin(imu_err_theta), np.cos(imu_err_theta))

    # Compute RMSE
    rmse_ekf_x = np.sqrt(np.mean(ekf_err_x**2))
    rmse_ekf_y = np.sqrt(np.mean(ekf_err_y**2))
    rmse_ekf_theta = np.sqrt(np.mean(ekf_err_theta**2))
    rmse_ekf_2d = np.sqrt(np.mean(ekf_err_x**2 + ekf_err_y**2))

    rmse_gps_x = np.sqrt(np.mean(gps_err_x**2))
    rmse_gps_y = np.sqrt(np.mean(gps_err_y**2))
    rmse_gps_2d = np.sqrt(np.mean(gps_err_x**2 + gps_err_y**2))

    rmse_imu_theta = np.sqrt(np.mean(imu_err_theta**2))

    return {
        'ekf': {'x': rmse_ekf_x, 'y': rmse_ekf_y, 'theta': rmse_ekf_theta, '2d': rmse_ekf_2d},
        'gps': {'x': rmse_gps_x, 'y': rmse_gps_y, '2d': rmse_gps_2d},
        'imu': {'theta': rmse_imu_theta}
    }


def print_rmse_table(rmse, output_dir):
    """Print and save RMSE comparison table."""
    table = f"""
╔════════════════════════════════════════════════════════════╗
║                    RMSE COMPARISON TABLE                   ║
╠════════════════╦═══════════════╦═══════════════════════════╣
║    Metric      ║   Raw Sensor  ║      EKF Estimate         ║
╠════════════════╬═══════════════╬═══════════════════════════╣
║  X (m)         ║    {rmse['gps']['x']:>8.4f}   ║         {rmse['ekf']['x']:>8.4f}            ║
║  Y (m)         ║    {rmse['gps']['y']:>8.4f}   ║         {rmse['ekf']['y']:>8.4f}            ║
║  2D (m)        ║    {rmse['gps']['2d']:>8.4f}   ║         {rmse['ekf']['2d']:>8.4f}            ║
║  Yaw (deg)     ║    {np.degrees(rmse['imu']['theta']):>8.4f}   ║         {np.degrees(rmse['ekf']['theta']):>8.4f}            ║
╚════════════════╩═══════════════╩═══════════════════════════╝
"""
    print(table)

    # Save to file
    with open(os.path.join(output_dir, 'rmse_table.txt'), 'w') as f:
        f.write(table)
    print("  ✓ rmse_table.txt")

    # Also save as CSV for easy parsing
    rmse_df = pd.DataFrame({
        'Metric': ['X (m)', 'Y (m)', '2D (m)', 'Yaw (rad)'],
        'Raw Sensor': [rmse['gps']['x'], rmse['gps']['y'], rmse['gps']['2d'], rmse['imu']['theta']],
        'EKF Estimate': [rmse['ekf']['x'], rmse['ekf']['y'], rmse['ekf']['2d'], rmse['ekf']['theta']]
    })
    rmse_df.to_csv(os.path.join(output_dir, 'rmse_data.csv'), index=False)
    print("  ✓ rmse_data.csv")


def print_summary(df, rmse):
    """Print summary statistics."""
    print("\n" + "="*60)
    print("EKF PERFORMANCE SUMMARY")
    print("="*60)
    print(f"Data points:     {len(df)}")
    print(f"Duration:        {df['time'].iloc[-1]:.1f} seconds")
    print(f"Sample rate:     ~{len(df)/df['time'].iloc[-1]:.1f} Hz")
    print()
    print("Improvement (EKF vs Raw Sensor):")
    print(f"  Position 2D:   {(1 - rmse['ekf']['2d']/rmse['gps']['2d'])*100:.1f}% reduction in RMSE")
    print("="*60)


def main():
    parser = argparse.ArgumentParser(description='Plot EKF performance data')
    parser.add_argument('csv_file', nargs='?', help='Path to CSV file')
    args = parser.parse_args()

    # Find CSV file
    if args.csv_file:
        csv_path = args.csv_file
    else:
        # Default to workspace ekf_logs directory
        log_dir = os.path.expanduser('~/repos/ekf_localization_turtlebot3/ekf_logs')
        if not os.path.exists(log_dir):
            print(f"Error: No log directory found at {log_dir}")
            sys.exit(1)
        
        csv_files = sorted([f for f in os.listdir(log_dir) if f.endswith('.csv') 
                           and f.startswith('ekf_data')])
        if not csv_files:
            print(f"Error: No CSV files found in {log_dir}")
            sys.exit(1)
        
        csv_path = os.path.join(log_dir, csv_files[-1])
        print(f"Using latest file: {csv_path}")

    # Load data
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points\n")

    # Create output directory
    output_dir = os.path.dirname(csv_path)

    # Generate plots
    print("Generating plots...")
    plot_trajectory(df, output_dir)
    plot_error_vs_time(df, output_dir)
    plot_covariance_consistency(df, output_dir)

    # Compute and display RMSE
    print("\nComputing RMSE...")
    rmse = compute_rmse(df)
    print_rmse_table(rmse, output_dir)

    # Summary
    print_summary(df, rmse)

    print(f"\nAll outputs saved to: {output_dir}")


if __name__ == '__main__':
    main()
