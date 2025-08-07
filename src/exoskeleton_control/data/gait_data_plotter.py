
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor
import os

def load_gait_data(filename):
    """Load gait data from JSON file"""
    try:
        with open(filename, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in '{filename}'.")
        return None

def extract_xy_data(data_array):
    """Extract X and Y coordinates from data array"""
    if not data_array or len(data_array[0]) < 2:
        return [], []

    x_coords = [point[0] for point in data_array]
    y_coords = [point[1] for point in data_array]
    return x_coords, y_coords

def create_interactive_plot(x_data, y_data, title, xlabel, ylabel, save_name=None):
    """Create an interactive plot with cursor functionality"""
    fig, ax = plt.subplots(figsize=(12, 8))

    # Plot the data
    line, = ax.plot(x_data, y_data, 'b-', linewidth=2, marker='o', markersize=3, alpha=0.7)

    # Set labels and title
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Add cursor for interactivity
    cursor = Cursor(ax, useblit=True, color='red', linewidth=1)

    # Add annotation for displaying values
    annot = ax.annotate('', xy=(0,0), xytext=(20,20), textcoords="offset points",
                       bbox=dict(boxstyle="round", fc="w", alpha=0.8),
                       arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    def on_hover(event):
        if event.inaxes == ax:
            # Find the closest point
            if len(x_data) > 0:
                distances = [(x_data[i] - event.xdata)**2 + (y_data[i] - event.ydata)**2 
                           for i in range(len(x_data))]
                closest_idx = distances.index(min(distances))

                # Update annotation
                annot.xy = (x_data[closest_idx], y_data[closest_idx])
                annot.set_text(f'Point {closest_idx}\nX: {x_data[closest_idx]:.4f}\nY: {y_data[closest_idx]:.4f}')
                annot.set_visible(True)
                fig.canvas.draw_idle()
        else:
            annot.set_visible(False)
            fig.canvas.draw_idle()

    # Connect the hover event
    fig.canvas.mpl_connect('motion_notify_event', on_hover)

    # Save the plot if requested
    if save_name:
        plt.savefig(f'{save_name}.png', dpi=300, bbox_inches='tight')
        print(f"Plot saved as {save_name}.png")

    return fig, ax

def create_xy_trajectory_plot(x_data, y_data, title, save_name=None):
    """Create X vs Y trajectory plot"""
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot trajectory
    line, = ax.plot(x_data, y_data, 'b-', linewidth=2, alpha=0.7)

    # Mark start and end points
    if len(x_data) > 0:
        ax.plot(x_data[0], y_data[0], 'go', markersize=10, label='Start')
        ax.plot(x_data[-1], y_data[-1], 'ro', markersize=10, label='End')

    # Add arrows to show direction
    if len(x_data) > 10:
        for i in range(0, len(x_data)-1, len(x_data)//10):
            dx = x_data[i+1] - x_data[i]
            dy = y_data[i+1] - y_data[i]
            ax.arrow(x_data[i], y_data[i], dx*0.5, dy*0.5, 
                    head_width=0.02, head_length=0.02, fc='red', ec='red', alpha=0.6)

    ax.set_xlabel('X Position', fontsize=12)
    ax.set_ylabel('Y Position', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect('equal', adjustable='box')

    # Add cursor
    cursor = Cursor(ax, useblit=True, color='red', linewidth=1)

    # Add annotation
    annot = ax.annotate('', xy=(0,0), xytext=(20,20), textcoords="offset points",
                       bbox=dict(boxstyle="round", fc="w", alpha=0.8),
                       arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    def on_hover(event):
        if event.inaxes == ax and len(x_data) > 0:
            distances = [(x_data[i] - event.xdata)**2 + (y_data[i] - event.ydata)**2 
                        for i in range(len(x_data))]
            closest_idx = distances.index(min(distances))

            annot.xy = (x_data[closest_idx], y_data[closest_idx])
            annot.set_text(f'Point {closest_idx}\nX: {x_data[closest_idx]:.4f}\nY: {y_data[closest_idx]:.4f}')
            annot.set_visible(True)
            fig.canvas.draw_idle()
        else:
            annot.set_visible(False)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('motion_notify_event', on_hover)

    if save_name:
        plt.savefig(f'{save_name}.png', dpi=300, bbox_inches='tight')
        print(f"Plot saved as {save_name}.png")

    return fig, ax

def create_time_series_plot(x_data, y_data, title_prefix, y_label, save_prefix=None):
    """Create a single time series plot for X and Y data."""
    time_points = list(range(len(x_data)))

    fig, ax = plt.subplots(figsize=(12, 8))

    ax.plot(time_points, x_data, 'b-', linewidth=2, marker='o', markersize=3, alpha=0.7, label='X')
    ax.plot(time_points, y_data, 'r-', linewidth=2, marker='o', markersize=3, alpha=0.7, label='Y')

    ax.set_xlabel('Time (samples)', fontsize=12)
    ax.set_ylabel(y_label, fontsize=12)
    ax.set_title(f'{title_prefix} vs Time', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Add cursor for interactivity
    cursor = Cursor(ax, useblit=True, color='green', linewidth=1)

    if save_prefix:
        plt.savefig(f'{save_prefix}_vs_time.png', dpi=300, bbox_inches='tight')
        print(f"Plot saved as {save_prefix}_vs_time.png")

    return fig, ax

def main():
    """Main function to process gait data and create plots"""
    # Load the JSON data
    filename = 'gait_data_39_1.json'
    data = load_gait_data(filename)

    if data is None:
        return

    # Extract the main object from the list
    if isinstance(data, list) and len(data) > 0:
        gait_data = data[0]
    else:
        print("Error: JSON data is not in the expected format (a list with one object).")
        return

    # Find position and velocity keys, excluding FR2
    position_keys = [key for key in gait_data.keys() if 'pos' in key and 'velocity' not in key and 'FR2' not in key]
    velocity_keys = [key for key in gait_data.keys() if 'velocity' in key and 'FR2' not in key]

    print(f"Found {len(position_keys)} position datasets:")
    for key in position_keys:
        print(f"  - {key}")

    print(f"\nFound {len(velocity_keys)} velocity datasets:")
    for key in velocity_keys:
        print(f"  - {key}")

    # Create output directory for saved plots
    output_dir = 'gait_plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"\nCreated output directory: {output_dir}")

    # Process each position dataset
    for pos_key in position_keys:
        print(f"\nProcessing {pos_key}...")
        pos_data = gait_data[pos_key]

        if pos_data and len(pos_data) > 0:
            x_pos, y_pos = extract_xy_data(pos_data)

            if x_pos and y_pos:
                # Create plots
                clean_name = pos_key.replace('_', ' ').title()
                save_prefix = os.path.join(output_dir, pos_key)

                # X vs Y trajectory plot
                create_xy_trajectory_plot(x_pos, y_pos, 
                                        f'{clean_name} - Trajectory (X vs Y)',
                                        f'{save_prefix}_trajectory')

                # Time series plots
                create_time_series_plot(x_pos, y_pos, 
                                       f'{clean_name} Position',
                                       'Position',
                                       save_prefix)

    # Process each velocity dataset
    for vel_key in velocity_keys:
        print(f"\nProcessing {vel_key}...")
        vel_data = gait_data[vel_key]

        if vel_data and len(vel_data) > 0:
            x_vel, y_vel = extract_xy_data(vel_data)

            if x_vel and y_vel:
                # Create plots
                clean_name = vel_key.replace('_', ' ').title()
                save_prefix = os.path.join(output_dir, vel_key)

                # X vs Y velocity plot
                create_xy_trajectory_plot(x_vel, y_vel,
                                        f'{clean_name} - Velocity (X vs Y)',
                                        f'{save_prefix}_trajectory')

                # Time series plots
                create_time_series_plot(x_vel, y_vel,
                                       f'{clean_name}',
                                       'Velocity',
                                       save_prefix)

    print("\n" + "="*60)
    print("INTERACTIVE FEATURES:")
    print("- Hover over any point to see its coordinates")
    print("- Red crosshair cursor follows mouse movement")
    print("- All plots are saved as high-resolution PNG files")
    print("- Use matplotlib's built-in zoom and pan tools")
    print("="*60)

    # Show all plots
    plt.show()

if __name__ == "__main__":
    main()
