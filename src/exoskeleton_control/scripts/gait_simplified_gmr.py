#!/usr/bin/env python3
"""
Simplified Gait Analysis GMR Trajectory Recovery Script

This script loads a trained TPGMM model and performs simplified trajectory recovery
following the logic from gait_example1time.ipynb more directly.
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from matplotlib.patches import Ellipse
import os
import sys

# Add TaskParameterizedGaussianMixtureModels to Python path
sys.path.append('TaskParameterizedGaussianMixtureModels')


def load_trained_model(model_path):
    """Load the trained TPGMM model from pickle file."""
    print(f"Loading trained model from: {model_path}")
    
    with open(model_path, 'rb') as f:
        model_data = pickle.load(f)
    
    print("Model loaded successfully!")
    print(f"Number of frames: {model_data['num_frames']}")
    print(f"Number of components: {model_data['n_components']}")
    print(f"Feature dimensions: {model_data['feature_dims']}")
    print(f"Feature names: {model_data['feature_names']}")
    
    return model_data


def extract_sample_trajectory(model_data, frame_idx=0, trajectory_idx=0):
    """Extract a sample trajectory from the training data."""
    
    # Get the sample trajectory
    sample_trajectory = model_data['all_trajectories'][frame_idx][trajectory_idx]
    
    print(f"Sample trajectory shape: {sample_trajectory.shape}")
    print(f"Time range: {sample_trajectory[:, 0].min():.3f} to {sample_trajectory[:, 0].max():.3f}")
    
    return sample_trajectory


def predict_using_tpgmm_gaussian_weights(tpgmm, time_input, feature_idx_to_predict):
    """
    Simplified prediction using TPGMM Gaussian weights.
    This follows the approach from the notebook more directly.
    """
    
    # For simplicity, we'll use the first frame (FR1) 
    frame_idx = 0
    
    # Get the means and covariances for the selected frame
    means = tpgmm.means_[frame_idx]  # Shape: (n_components, n_features)
    covariances = tpgmm.covariances_[frame_idx]  # Shape: (n_components, n_features, n_features)
    weights = tpgmm.weights_  # Shape: (n_components,)
    
    n_points = len(time_input)
    n_output_features = len(feature_idx_to_predict)
    
    predicted_output = np.zeros((n_points, n_output_features))
    predicted_covariance = np.zeros((n_points, n_output_features, n_output_features))
    
    # Time index (input)
    time_idx = 0
    
    for i, t in enumerate(time_input):
        # Calculate responsibilities (h) for each component at this time point
        responsibilities = np.zeros(tpgmm._n_components)
        
        for k in range(tpgmm._n_components):
            # Simple Gaussian evaluation at time t
            mean_time = means[k, time_idx]
            var_time = covariances[k, time_idx, time_idx]
            
            # Gaussian probability at time t
            prob = np.exp(-0.5 * ((t - mean_time) ** 2) / var_time) / np.sqrt(2 * np.pi * var_time)
            responsibilities[k] = weights[k] * prob
        
        # Normalize responsibilities
        responsibilities /= (np.sum(responsibilities) + 1e-10)
        
        # Predict output features using Gaussian Mixture Regression
        pred_mean = np.zeros(n_output_features)
        pred_cov = np.zeros((n_output_features, n_output_features))
        
        for k in range(tpgmm._n_components):
            # Extract means and covariances for input and output
            mu_i = means[k, time_idx]  # input mean
            mu_o = means[k, feature_idx_to_predict]  # output mean
            
            sigma_ii = covariances[k, time_idx, time_idx]  # input-input covariance
            sigma_io = covariances[k, time_idx, feature_idx_to_predict]  # input-output covariance  
            sigma_oo = covariances[k][np.ix_(feature_idx_to_predict, feature_idx_to_predict)]  # output-output covariance
            
            # GMR prediction for component k
            pred_mean_k = mu_o + (sigma_io.T / sigma_ii) * (t - mu_i)
            pred_cov_k = sigma_oo - np.outer(sigma_io, sigma_io) / sigma_ii
            
            # Weight by responsibility
            pred_mean += responsibilities[k] * pred_mean_k
            pred_cov += responsibilities[k] * (pred_cov_k + np.outer(pred_mean_k, pred_mean_k))
        
        # Correct covariance calculation
        pred_cov -= np.outer(pred_mean, pred_mean)
        
        predicted_output[i] = pred_mean
        predicted_covariance[i] = pred_cov
    
    return predicted_output, predicted_covariance


def apply_pca_to_features(trajectories, feature_names, n_components=2):
    """Apply PCA to extract principal components from high-dimensional features."""
    
    # Separate position and velocity features (exclude time)
    pos_indices = [i for i, name in enumerate(feature_names) if 'pos' in name]
    vel_indices = [i for i, name in enumerate(feature_names) if 'vel' in name]
    
    print(f"Position feature indices: {pos_indices}")
    print(f"Velocity feature indices: {vel_indices}")
    
    results = {}
    
    # Apply PCA to position features
    if len(pos_indices) > 0:
        pos_data = trajectories[:, pos_indices]
        pca_pos = PCA(n_components=min(n_components, pos_data.shape[1]))
        pos_pca = pca_pos.fit_transform(pos_data)
        
        results['position'] = {
            'pca_data': pos_pca,
            'pca_model': pca_pos,
            'explained_variance': pca_pos.explained_variance_ratio_,
            'feature_indices': pos_indices,
            'original_data': pos_data
        }
        
        print(f"Position PCA explained variance: {pca_pos.explained_variance_ratio_}")
    
    # Apply PCA to velocity features  
    if len(vel_indices) > 0:
        vel_data = trajectories[:, vel_indices]
        pca_vel = PCA(n_components=min(n_components, vel_data.shape[1]))
        vel_pca = pca_vel.fit_transform(vel_data)
        
        results['velocity'] = {
            'pca_data': vel_pca,
            'pca_model': pca_vel,
            'explained_variance': pca_vel.explained_variance_ratio_,
            'feature_indices': vel_indices,
            'original_data': vel_data
        }
        
        print(f"Velocity PCA explained variance: {pca_vel.explained_variance_ratio_}")
    
    return results


def plot_gaussian_models(model_data, predicted_trajectory=None, save_dir="plots"):
    """Plot Gaussian models with means and covariance ellipses for position and velocity dimensions."""
    
    os.makedirs(save_dir, exist_ok=True)
    
    tpgmm = model_data['tpgmm']
    feature_names = model_data['feature_names']
    
    # Use the first frame (FR1)
    frame_idx = 0
    means = tpgmm.means_[frame_idx]  # Shape: (n_components, n_features)
    covariances = tpgmm.covariances_[frame_idx]  # Shape: (n_components, n_features, n_features)
    weights = tpgmm.weights_  # Shape: (n_components,)
    
    # Position dimensions: [1, 2, 5, 6] -> [right_ankle_pos_x, right_ankle_pos_y, left_ankle_pos_x, left_ankle_pos_y]
    pos_dims = [1, 2, 5, 6]
    # Velocity dimensions: [3, 4, 7, 8] -> [right_ankle_vel_x, right_ankle_vel_y, left_ankle_vel_x, left_ankle_vel_y]
    vel_dims = [3, 4, 7, 8]
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    
    # Define colors for each Gaussian component
    colors = plt.cm.tab10(np.linspace(0, 1, tpgmm._n_components))
    
    # Plot recovered trajectory if provided
    if predicted_trajectory is not None:
        # Right ankle positions trajectory (dims 0, 1 in predicted_trajectory correspond to dims 1, 2)
        ax1.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], 'k-', 
                linewidth=3, alpha=0.7, label='GMR Recovery', zorder=10)
        ax1.plot(predicted_trajectory[0, 0], predicted_trajectory[0, 1], 'ko', 
                markersize=8, label='Start', zorder=11)
        ax1.plot(predicted_trajectory[-1, 0], predicted_trajectory[-1, 1], 'ks', 
                markersize=8, label='End', zorder=11)
    
    # Plot position Gaussians
    # Right ankle positions (dims 1, 2)
    for k in range(tpgmm._n_components):
        mean_pos = means[k, [1, 2]]  # [right_ankle_pos_x, right_ankle_pos_y]
        cov_pos = covariances[k][np.ix_([1, 2], [1, 2])]  # 2x2 covariance matrix
        
        # Plot mean as point
        ax1.scatter(mean_pos[0], mean_pos[1], c=[colors[k]], s=100*weights[k]*10, 
                   alpha=0.8, edgecolors='black', linewidth=1, label=f'Component {k+1}', zorder=5)
        
        # Plot covariance as ellipse
        eigenvals, eigenvecs = np.linalg.eigh(cov_pos)
        angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        width, height = 2 * np.sqrt(eigenvals)  # 1-sigma ellipse
        
        ellipse = Ellipse(mean_pos, width, height, angle=angle, 
                         facecolor=colors[k], alpha=0.3, edgecolor=colors[k], linewidth=2, zorder=1)
        ax1.add_patch(ellipse)
    
    ax1.set_title('Right Ankle Position Gaussians\n(Dimensions 1, 2)', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Right Ankle X Position (m)')
    ax1.set_ylabel('Right Ankle Y Position (m)')
    ax1.grid(True, alpha=0.3)
    ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax1.axis('equal')
    
    # Plot recovered trajectory if provided
    if predicted_trajectory is not None:
        # Left ankle positions trajectory (dims 4, 5 in predicted_trajectory correspond to dims 5, 6)
        ax2.plot(predicted_trajectory[:, 4], predicted_trajectory[:, 5], 'k-', 
                linewidth=3, alpha=0.7, label='GMR Recovery', zorder=10)
        ax2.plot(predicted_trajectory[0, 4], predicted_trajectory[0, 5], 'ko', 
                markersize=8, label='Start', zorder=11)
        ax2.plot(predicted_trajectory[-1, 4], predicted_trajectory[-1, 5], 'ks', 
                markersize=8, label='End', zorder=11)
    
    # Left ankle positions (dims 5, 6)
    for k in range(tpgmm._n_components):
        mean_pos = means[k, [5, 6]]  # [left_ankle_pos_x, left_ankle_pos_y]
        cov_pos = covariances[k][np.ix_([5, 6], [5, 6])]  # 2x2 covariance matrix
        
        # Plot mean as point
        ax2.scatter(mean_pos[0], mean_pos[1], c=[colors[k]], s=100*weights[k]*10, 
                   alpha=0.8, edgecolors='black', linewidth=1, zorder=5)
        
        # Plot covariance as ellipse
        eigenvals, eigenvecs = np.linalg.eigh(cov_pos)
        angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        width, height = 2 * np.sqrt(eigenvals)  # 1-sigma ellipse
        
        ellipse = Ellipse(mean_pos, width, height, angle=angle, 
                         facecolor=colors[k], alpha=0.3, edgecolor=colors[k], linewidth=2, zorder=1)
        ax2.add_patch(ellipse)
    
    ax2.set_title('Left Ankle Position Gaussians\n(Dimensions 5, 6)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Left Ankle X Position (m)')
    ax2.set_ylabel('Left Ankle Y Position (m)')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # Plot recovered trajectory if provided
    if predicted_trajectory is not None:
        # Right ankle velocities trajectory (dims 2, 3 in predicted_trajectory correspond to dims 3, 4)
        ax3.plot(predicted_trajectory[:, 2], predicted_trajectory[:, 3], 'k-', 
                linewidth=3, alpha=0.7, label='GMR Recovery', zorder=10)
        ax3.plot(predicted_trajectory[0, 2], predicted_trajectory[0, 3], 'ko', 
                markersize=8, label='Start', zorder=11)
        ax3.plot(predicted_trajectory[-1, 2], predicted_trajectory[-1, 3], 'ks', 
                markersize=8, label='End', zorder=11)
    
    # Plot velocity Gaussians
    # Right ankle velocities (dims 3, 4)
    for k in range(tpgmm._n_components):
        mean_vel = means[k, [3, 4]]  # [right_ankle_vel_x, right_ankle_vel_y]
        cov_vel = covariances[k][np.ix_([3, 4], [3, 4])]  # 2x2 covariance matrix
        
        # Plot mean as point
        ax3.scatter(mean_vel[0], mean_vel[1], c=[colors[k]], s=100*weights[k]*10, 
                   alpha=0.8, edgecolors='black', linewidth=1, zorder=5)
        
        # Plot covariance as ellipse
        eigenvals, eigenvecs = np.linalg.eigh(cov_vel)
        angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        width, height = 2 * np.sqrt(eigenvals)  # 1-sigma ellipse
        
        ellipse = Ellipse(mean_vel, width, height, angle=angle, 
                         facecolor=colors[k], alpha=0.3, edgecolor=colors[k], linewidth=2, zorder=1)
        ax3.add_patch(ellipse)
    
    ax3.set_title('Right Ankle Velocity Gaussians\n(Dimensions 3, 4)', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Right Ankle X Velocity (m/s)')
    ax3.set_ylabel('Right Ankle Y Velocity (m/s)')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # Plot recovered trajectory if provided
    if predicted_trajectory is not None:
        # Left ankle velocities trajectory (dims 6, 7 in predicted_trajectory correspond to dims 7, 8)
        ax4.plot(predicted_trajectory[:, 6], predicted_trajectory[:, 7], 'k-', 
                linewidth=3, alpha=0.7, label='GMR Recovery', zorder=10)
        ax4.plot(predicted_trajectory[0, 6], predicted_trajectory[0, 7], 'ko', 
                markersize=8, label='Start', zorder=11)
        ax4.plot(predicted_trajectory[-1, 6], predicted_trajectory[-1, 7], 'ks', 
                markersize=8, label='End', zorder=11)
    
    # Left ankle velocities (dims 7, 8)
    for k in range(tpgmm._n_components):
        mean_vel = means[k, [7, 8]]  # [left_ankle_vel_x, left_ankle_vel_y]
        cov_vel = covariances[k][np.ix_([7, 8], [7, 8])]  # 2x2 covariance matrix
        
        # Plot mean as point
        ax4.scatter(mean_vel[0], mean_vel[1], c=[colors[k]], s=100*weights[k]*10, 
                   alpha=0.8, edgecolors='black', linewidth=1, zorder=5)
        
        # Plot covariance as ellipse
        eigenvals, eigenvecs = np.linalg.eigh(cov_vel)
        angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        width, height = 2 * np.sqrt(eigenvals)  # 1-sigma ellipse
        
        ellipse = Ellipse(mean_vel, width, height, angle=angle, 
                         facecolor=colors[k], alpha=0.3, edgecolor=colors[k], linewidth=2, zorder=1)
        ax4.add_patch(ellipse)
    
    ax4.set_title('Left Ankle Velocity Gaussians\n(Dimensions 7, 8)', fontsize=14, fontweight='bold')
    ax4.set_xlabel('Left Ankle X Velocity (m/s)')
    ax4.set_ylabel('Left Ankle Y Velocity (m/s)')
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    plt.tight_layout()
    plt.savefig(f'{save_dir}/gaussian_models_visualization.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Gaussian models visualization saved to {save_dir}/gaussian_models_visualization.png")


def plot_recovery_results(original_trajectory, predicted_trajectory, pca_results, feature_names, save_dir="plots"):
    """Plot trajectory recovery results with PCA visualization."""
    
    os.makedirs(save_dir, exist_ok=True)
    
    # Plot 1: Time series comparison
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    
    time_data = original_trajectory[:, 0]
    
    # Plot key features over time
    feature_plots = [
        (1, 'Right Ankle X Pos'),
        (2, 'Right Ankle Y Pos'), 
        (3, 'Right Ankle X Vel'),
        (4, 'Right Ankle Y Vel')
    ]
    
    for idx, (feat_idx, title) in enumerate(feature_plots):
        row = idx // 2
        col = idx % 2
        
        axes[row, col].plot(time_data, original_trajectory[:, feat_idx], 'b-', label='Original', linewidth=2)
        axes[row, col].plot(time_data, predicted_trajectory[:, feat_idx-1], 'r--', label='Predicted', linewidth=2)
        axes[row, col].set_title(title)
        axes[row, col].set_xlabel('Time')
        axes[row, col].set_ylabel('Value')
        axes[row, col].legend()
        axes[row, col].grid(True, alpha=0.3)
    
    # PCA plots
    if 'position' in pca_results:
        pca_pos = pca_results['position']['pca_data']
        axes[0, 2].plot(pca_pos[:, 0], pca_pos[:, 1], 'g-', linewidth=2)
        axes[0, 2].set_title('Position PCA (First 2 Components)')
        axes[0, 2].set_xlabel(f'PC1 ({pca_results["position"]["explained_variance"][0]:.2%} variance)')
        axes[0, 2].set_ylabel(f'PC2 ({pca_results["position"]["explained_variance"][1]:.2%} variance)')
        axes[0, 2].grid(True, alpha=0.3)
        axes[0, 2].axis('equal')
    
    if 'velocity' in pca_results:
        pca_vel = pca_results['velocity']['pca_data']
        axes[0, 3].plot(pca_vel[:, 0], pca_vel[:, 1], 'm-', linewidth=2)
        axes[0, 3].set_title('Velocity PCA (First 2 Components)')
        axes[0, 3].set_xlabel(f'PC1 ({pca_results["velocity"]["explained_variance"][0]:.2%} variance)')
        axes[0, 3].set_ylabel(f'PC2 ({pca_results["velocity"]["explained_variance"][1]:.2%} variance)')
        axes[0, 3].grid(True, alpha=0.3)
        axes[0, 3].axis('equal')
    
    # Additional PCA comparison plots
    if 'position' in pca_results:
        # Plot original vs PCA reconstruction for positions
        pos_original = pca_results['position']['original_data']
        axes[1, 2].plot(pos_original[:, 0], pos_original[:, 1], 'b-', label='Right Ankle', linewidth=2)
        axes[1, 2].plot(pos_original[:, 2], pos_original[:, 3], 'r-', label='Left Ankle', linewidth=2)
        axes[1, 2].set_title('Original Position Trajectories')
        axes[1, 2].set_xlabel('X Position (m)')
        axes[1, 2].set_ylabel('Y Position (m)')
        axes[1, 2].legend()
        axes[1, 2].grid(True, alpha=0.3)
        axes[1, 2].axis('equal')
    
    if 'velocity' in pca_results:
        # Plot original vs PCA reconstruction for velocities
        vel_original = pca_results['velocity']['original_data']
        axes[1, 3].plot(vel_original[:, 0], vel_original[:, 1], 'b-', label='Right Ankle', linewidth=2)
        axes[1, 3].plot(vel_original[:, 2], vel_original[:, 3], 'r-', label='Left Ankle', linewidth=2)
        axes[1, 3].set_title('Original Velocity Trajectories')
        axes[1, 3].set_xlabel('X Velocity (m/s)')
        axes[1, 3].set_ylabel('Y Velocity (m/s)')
        axes[1, 3].legend()
        axes[1, 3].grid(True, alpha=0.3)
        axes[1, 3].axis('equal')
    
    plt.tight_layout()
    plt.savefig(f'{save_dir}/simplified_gmr_recovery_results.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Recovery results plots saved to {save_dir}/ directory")


def main():
    """Main simplified GMR trajectory recovery pipeline."""
    print("=== Simplified Gait Analysis GMR Trajectory Recovery ===")
    
    # Configuration
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)  # Go up from scripts to package root
    pkls_dir = os.path.join(package_dir, "pkls")
    model_path = os.path.join(pkls_dir, "gait_tpgmm_model_final.pkl")
    
    # Step 1: Load trained model
    print("\nStep 1: Loading trained TPGMM model...")
    model_data = load_trained_model(model_path)
    
    tpgmm = model_data['tpgmm']
    feature_names = model_data['feature_names']
    
    # Step 2: Extract sample trajectory
    print("\nStep 2: Extracting sample trajectory...")
    sample_trajectory = extract_sample_trajectory(model_data, frame_idx=0, trajectory_idx=0)
    
    # Step 3: Prepare input data (time vector)
    print("\nStep 3: Preparing time input...")
    time_input = sample_trajectory[:, 0]  # Time column
    print(f"Time input shape: {time_input.shape}")
    
    # Step 4: Define output features to predict (exclude time)
    output_feature_indices = list(range(1, len(feature_names)))  # [1, 2, 3, 4, 5, 6, 7, 8]
    print(f"Output feature indices: {output_feature_indices}")
    print(f"Output features: {[feature_names[i] for i in output_feature_indices]}")
    
    # Step 5: Perform simplified GMR prediction
    print("\nStep 5: Performing simplified GMR prediction...")
    predicted_output, predicted_covariance = predict_using_tpgmm_gaussian_weights(
        tpgmm, time_input, output_feature_indices
    )
    
    print(f"Predicted output shape: {predicted_output.shape}")
    print(f"Predicted covariance shape: {predicted_covariance.shape}")
    
    # Step 6: Apply PCA to extract principal components
    print("\nStep 6: Applying PCA for dimensionality reduction...")
    pca_results = apply_pca_to_features(predicted_output, feature_names[1:])  # Exclude time from PCA
    
    # Step 7: Plot Gaussian models visualization
    print("\nStep 7: Creating Gaussian models visualization...")
    plot_gaussian_models(model_data, predicted_output)
    
    # Step 8: Plot recovery results
    print("\nStep 8: Creating recovery results visualizations...")
    plot_recovery_results(sample_trajectory, predicted_output, pca_results, feature_names)
    
    # Step 9: Save recovered trajectory
    print("\nStep 9: Saving recovered trajectory...")
    recovery_data = {
        'original_trajectory': sample_trajectory,
        'predicted_trajectory': predicted_output,
        'prediction_covariance': predicted_covariance,
        'pca_results': pca_results,
        'feature_names': feature_names,
        'time_input': time_input,
        'output_feature_indices': output_feature_indices
    }
    
    recovery_save_path = os.path.join(pkls_dir, "gait_simplified_recovery.pkl")
    with open(recovery_save_path, 'wb') as f:
        pickle.dump(recovery_data, f)
        
    print(f"Recovery data saved to: {recovery_save_path}")
    print("\n=== Simplified GMR Trajectory Recovery Complete ===")


if __name__ == "__main__":
    main()