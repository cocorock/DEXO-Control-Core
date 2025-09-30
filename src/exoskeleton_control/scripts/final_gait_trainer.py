#!/usr/bin/env python3
"""
Final Gait Analysis TPGMM Training Script

This script trains a Task Parameterized Gaussian Mixture Model (TPGMM) with 3 frames of reference
for gait analysis using ankle position and velocity data from both legs.
Following the structure from gait_example1time.ipynb with plotting functionality.
"""

import json
import numpy as np
import pickle
import sys
import os
from pathlib import Path
import matplotlib.pyplot as plt

# Import the TPGMM implementation

# Add workspace root to Python path to access tpgmm library
workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if workspace_root not in sys.path:
    sys.path.insert(0, workspace_root)

from tpgmm.tpgmm.tpgmm import TPGMM


def load_gait_data(json_path):
    """Load and process gait data from JSON file following notebook structure."""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # Get kinematics data from all 3 frames
    frames = ['FR1', 'FR2', 'FR3']
    all_trajectories = []
    
    # Process each frame of reference
    for frame_idx, frame in enumerate(frames):
        kinematics = data['kinematics_data'][frame]['right_leg_kinematics']
        num_cycles = len(kinematics)
        interpolation_points = data['parameters']['interpolation_points']
        
        if frame_idx == 0:  # Print info only once
            print(f"Processing {num_cycles} gait cycles from {len(frames)} frames")
            print(f"Each cycle has {interpolation_points} interpolation points")
        
        frame_trajectories = []
        
        for cycle_idx, cycle_data in enumerate(kinematics):
            # Extract ankle data for both legs
            right_ankle_pos = np.array(cycle_data['right_ankle_pos'])
            right_ankle_vel = np.array(cycle_data['right_ankle_vel']) 
            left_ankle_pos = np.array(cycle_data['left_ankle_pos'])
            left_ankle_vel = np.array(cycle_data['left_ankle_vel'])
            
            # Create time vector (normalized 0-1)
            time_vector = np.linspace(0, 1, interpolation_points)
            
            # Create 9-dimensional feature space as described in summary:
            # [time, right_ankle_pos_x, right_ankle_pos_y, right_ankle_vel_x, right_ankle_vel_y,
            #  left_ankle_pos_x, left_ankle_pos_y, left_ankle_vel_x, left_ankle_vel_y]
            trajectory = np.column_stack([
                time_vector,               # time (0-1)
                right_ankle_pos[0, :],     # right ankle X position
                right_ankle_pos[1, :],     # right ankle Y position
                right_ankle_vel[0, :],     # right ankle X velocity
                right_ankle_vel[1, :],     # right ankle Y velocity
                left_ankle_pos[0, :],      # left ankle X position
                left_ankle_pos[1, :],      # left ankle Y position
                left_ankle_vel[0, :],      # left ankle X velocity
                left_ankle_vel[1, :]       # left ankle Y velocity
            ])
            
            frame_trajectories.append(trajectory)
        
        # Convert to numpy array and store
        frame_trajectories = np.array(frame_trajectories)
        all_trajectories.append(frame_trajectories)
        print(f"{frame} trajectories shape: {frame_trajectories.shape}")
    
    return all_trajectories


def plot_input_trajectories(all_trajectories):
    """Plot input trajectories for all frames of reference."""
    frames = ['FR1', 'FR2', 'FR3']
    frame_colors = {'FR1': 'blue', 'FR2': 'red', 'FR3': 'green'}
    
    # Create plots directory
    os.makedirs('plots', exist_ok=True)
    
    # Plot ankle positions - merged into one plot per side
    fig, (ax_right, ax_left) = plt.subplots(1, 2, figsize=(16, 8))
    
    for frame_idx, (frame_name, trajectories) in enumerate(zip(frames, all_trajectories)):
        color = frame_colors[frame_name]
        
        # Right ankle positions - all frames in one plot
        for traj_idx, traj in enumerate(trajectories):
            alpha = 0.6 if traj_idx > 0 else 0.8
            linewidth = 1.5 if traj_idx == 0 else 1
            label = frame_name if traj_idx == 0 else ""
            ax_right.plot(traj[:, 1], traj[:, 2], color=color, alpha=alpha, linewidth=linewidth, label=label)
        
        # Left ankle positions - all frames in one plot
        for traj_idx, traj in enumerate(trajectories):
            alpha = 0.6 if traj_idx > 0 else 0.8
            linewidth = 1.5 if traj_idx == 0 else 1
            label = frame_name if traj_idx == 0 else ""
            ax_left.plot(traj[:, 5], traj[:, 6], color=color, alpha=alpha, linewidth=linewidth, label=label)
    
    ax_right.set_title('Right Ankle Position Trajectories\nAll Frames Combined', fontsize=14, fontweight='bold')
    ax_right.set_xlabel('X Position (m)')
    ax_right.set_ylabel('Y Position (m)')
    ax_right.grid(True, alpha=0.3)
    ax_right.legend()
    ax_right.axis('equal')
    
    ax_left.set_title('Left Ankle Position Trajectories\nAll Frames Combined', fontsize=14, fontweight='bold')
    ax_left.set_xlabel('X Position (m)')
    ax_left.set_ylabel('Y Position (m)')
    ax_left.grid(True, alpha=0.3)
    ax_left.legend()
    ax_left.axis('equal')
    
    plt.tight_layout()
    plt.savefig('plots/input_trajectories_positions.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Plot ankle velocities - merged into one plot per side
    fig, (ax_right, ax_left) = plt.subplots(1, 2, figsize=(16, 8))
    
    for frame_idx, (frame_name, trajectories) in enumerate(zip(frames, all_trajectories)):
        color = frame_colors[frame_name]
        
        # Right ankle velocities - all frames in one plot
        for traj_idx, traj in enumerate(trajectories):
            alpha = 0.6 if traj_idx > 0 else 0.8
            linewidth = 1.5 if traj_idx == 0 else 1
            label = frame_name if traj_idx == 0 else ""
            ax_right.plot(traj[:, 3], traj[:, 4], color=color, alpha=alpha, linewidth=linewidth, label=label)
        
        # Left ankle velocities - all frames in one plot
        for traj_idx, traj in enumerate(trajectories):
            alpha = 0.6 if traj_idx > 0 else 0.8
            linewidth = 1.5 if traj_idx == 0 else 1
            label = frame_name if traj_idx == 0 else ""
            ax_left.plot(traj[:, 7], traj[:, 8], color=color, alpha=alpha, linewidth=linewidth, label=label)
    
    ax_right.set_title('Right Ankle Velocity Trajectories\nAll Frames Combined', fontsize=14, fontweight='bold')
    ax_right.set_xlabel('X Velocity (m/s)')
    ax_right.set_ylabel('Y Velocity (m/s)')
    ax_right.grid(True, alpha=0.3)
    ax_right.legend()
    ax_right.axis('equal')
    
    ax_left.set_title('Left Ankle Velocity Trajectories\nAll Frames Combined', fontsize=14, fontweight='bold')
    ax_left.set_xlabel('X Velocity (m/s)')
    ax_left.set_ylabel('Y Velocity (m/s)')
    ax_left.grid(True, alpha=0.3)
    ax_left.legend()
    ax_left.axis('equal')
    
    plt.tight_layout()
    plt.savefig('plots/input_trajectories_velocities.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("Input trajectory plots saved to plots/ directory")


def prepare_tpgmm_data(all_trajectories):
    """Prepare data for TPGMM training following notebook structure."""
    # Convert list of 3 trajectory arrays to stacked format
    # all_trajectories: list of 3 arrays, each with shape (num_cycles, num_points, num_features)
    
    num_frames = len(all_trajectories)
    num_trajectories, num_samples, num_features = all_trajectories[0].shape
    
    # Stack the data for all frames
    reshaped_trajectories = np.stack(all_trajectories, axis=0)
    
    # Reshape to (num_frames, num_trajectories * num_samples, num_features)
    reshaped_trajectories = reshaped_trajectories.reshape(num_frames, num_trajectories * num_samples, num_features)
    
    print(f"Reshaped trajectories shape: {reshaped_trajectories.shape}")
    print(f"Features: [time, right_ankle_pos_x, right_ankle_pos_y, right_ankle_vel_x, right_ankle_vel_y,")
    print(f"          left_ankle_pos_x, left_ankle_pos_y, left_ankle_vel_x, left_ankle_vel_y]")
    
    return reshaped_trajectories


def find_optimal_components(reshaped_trajectories, component_range=(2, 19)):
    """Find optimal number of components using BIC and AIC scores with visualization."""
    print("\nFinding optimal number of components...")
    
    # Create plots directory
    os.makedirs('plots', exist_ok=True)
    
    n_components_list = []
    bic_scores = []
    aic_scores = []
    
    best_n_components = None
    lowest_aic_score = float('inf')
    
    # Loop through n_components
    for n_components in range(component_range[0], component_range[1]):
        print(f'Fitting TPGMM with n_components={n_components}...')
        
        # Define the TPGMM model with the current n_components
        tpgmm = TPGMM(n_components=n_components, verbose=False, threshold=1e-5, reg_factor=1e-8)
        
        # Fit the model with the trajectories
        tpgmm.fit(reshaped_trajectories)
        
        # Calculate the BIC and AIC scores
        bic_score = tpgmm.bic(reshaped_trajectories)
        aic_score = tpgmm.aic(reshaped_trajectories)
        
        print(f'n_components={n_components}: BIC={bic_score:.2f}, AIC={aic_score:.2f}')
        
        # Store results
        n_components_list.append(n_components)
        bic_scores.append(bic_score)
        aic_scores.append(aic_score)
        
        # Update the best n_components and lowest AIC score if the current AIC is lower
        if aic_score < lowest_aic_score:
            lowest_aic_score = aic_score
            best_n_components = n_components
    
    # Plot BIC and AIC scores
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # BIC plot
    ax1.plot(n_components_list, bic_scores, 'bo-', linewidth=2, markersize=8, label='BIC')
    ax1.axvline(x=best_n_components, color='red', linestyle='--', linewidth=2, 
               label=f'AIC Selected (n={best_n_components})')
    ax1.set_xlabel('Number of Components', fontsize=12, fontweight='bold')
    ax1.set_ylabel('BIC Score', fontsize=12, fontweight='bold')
    ax1.set_title('Bayesian Information Criterion (BIC)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=12)
    ax1.tick_params(labelsize=11)
    
    # AIC plot (best_n_components is now the AIC-selected value)
    ax2.plot(n_components_list, aic_scores, 'go-', linewidth=2, markersize=8, label='AIC')
    ax2.axvline(x=best_n_components, color='red', linestyle='--', linewidth=2, 
               label=f'Optimal (n={best_n_components})')
    ax2.set_xlabel('Number of Components', fontsize=12, fontweight='bold')
    ax2.set_ylabel('AIC Score', fontsize=12, fontweight='bold')
    ax2.set_title('Akaike Information Criterion (AIC)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=12)
    ax2.tick_params(labelsize=11)
    
    plt.tight_layout()
    plt.savefig('plots/model_selection_criteria.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    # Find BIC optimal for comparison
    best_bic_idx = np.argmin(bic_scores)
    best_bic_components = n_components_list[best_bic_idx]
    
    print(f'\n=== Model Selection Results ===')
    print(f'AIC optimal n_components: {best_n_components} (AIC = {lowest_aic_score:.2f}) [SELECTED]')
    print(f'BIC optimal n_components: {best_bic_components} (BIC = {min(bic_scores):.2f}) [reference only]')
    print(f'Model selection plot saved to plots/model_selection_criteria.png')
    
    # Return results dictionary
    return {
        'best_n_components': best_n_components,  # Now AIC-based
        'best_aic_components': best_n_components,  # Same as above
        'best_bic_components': best_bic_components,  # For reference
        'n_components_list': n_components_list,
        'bic_scores': bic_scores,
        'aic_scores': aic_scores,
        'lowest_aic_score': lowest_aic_score,
        'lowest_bic_score': min(bic_scores)
    }


def main():
    """Main training pipeline."""
    print("=== Final Gait Analysis TPGMM Training ===")
    
    # Configuration
    json_path = "/home/jemajuinta/ws/Gait-analysis-coupled/alpha/Gait Data/4D/gait_analysis_export_subject35.json"
    
    # Create pkls directory if it doesn't exist
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)  # Go up from scripts to package root
    pkls_dir = os.path.join(package_dir, "pkls")
    os.makedirs(pkls_dir, exist_ok=True)
    
    model_save_path = os.path.join(pkls_dir, "gait_tpgmm_model_final.pkl")
    
    print(f"JSON data path: {json_path}")
    print(f"Model save path: {model_save_path}")
    print()
    
    # Step 1: Load and process data
    print("Step 1: Loading gait data...")
    all_trajectories = load_gait_data(json_path)
    
    # Step 2: Plot input trajectories
    print("\nStep 2: Plotting input trajectories...")
    plot_input_trajectories(all_trajectories)
    
    # Step 3: Prepare TPGMM data
    print("\nStep 3: Preparing TPGMM data...")
    reshaped_trajectories = prepare_tpgmm_data(all_trajectories)
    
    # Step 4: Find optimal components using BIC and AIC
    print("\nStep 4: Finding optimal components using BIC and AIC...")
    model_selection_results = find_optimal_components(reshaped_trajectories, component_range=(2, 31))
    best_n_components = model_selection_results['best_n_components']
    
    # Step 5: Train final TPGMM with optimal components
    print(f"\nStep 5: Training final TPGMM with n_components={best_n_components}...")
    tpgmm = TPGMM(n_components=best_n_components, verbose=True, reg_factor=1e-8, threshold=1e-5)
    tpgmm.fit(reshaped_trajectories)
    
    print("TPGMM training completed!")
    
    # Step 6: Save model
    print("\nStep 6: Saving model...")
    model_data = {
        'tpgmm': tpgmm,
        'n_components': best_n_components,
        'weights_': tpgmm.weights_,
        'means_': tpgmm.means_,
        'covariances_': tpgmm.covariances_,
        'all_trajectories': all_trajectories,
        'reshaped_trajectories': reshaped_trajectories,
        'model_selection_results': model_selection_results,
        'feature_names': [
            'time', 'right_ankle_pos_x', 'right_ankle_pos_y', 'right_ankle_vel_x', 'right_ankle_vel_y',
            'left_ankle_pos_x', 'left_ankle_pos_y', 'left_ankle_vel_x', 'left_ankle_vel_y'
        ],
        'frame_names': ['FR1', 'FR2', 'FR3'],
        'num_frames': 3,
        'feature_dims': 9
    }
    
    with open(model_save_path, 'wb') as f:
        pickle.dump(model_data, f)
        
    print(f"Model saved to: {model_save_path}")
    print("\n=== Training Complete ===")
    print("The model can now be loaded for Gaussian Mixture Regression (GMR).")


if __name__ == "__main__":
    main()