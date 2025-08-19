import json
import numpy as np
from sklearn.mixture import GaussianMixture
from sklearn.decomposition import PCA
import joblib
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.colors as mcolors
from scipy.stats import chi2
import os

class TPGMMGaitTrainer:
    def __init__(self):
        """
        TP-GMM Trainer for gait data with 2 frames of reference
        
        Frame dimensions:
        - Position: 2D (x, y)
        - Velocity: 2D (x, y) 
        - Orientation: 1D
        Total per frame: 5D
        """
        self.num_frames = 2  # FR1 + FR2
        
        # Dimensions per frame
        self.dims = {
            'position': 2,    # x, y
            'velocity': 2,    # vx, vy
            'orientation': 1  # theta
        }
        self.point_dim = sum(self.dims.values())  # 5D per frame
        self.total_dim = self.point_dim * self.num_frames  # 10D total
        
    def load_gait_demonstrations(self, json_file: str) -> List[np.ndarray]:
        """
        Load gait demonstrations from JSON file
        
        Args:
            json_file: Path to JSON file with gait data
            
        Returns:
            List of demonstration arrays
        """
        print(f"Loading gait demonstrations from: {json_file}")
        
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            demonstrations = []
            
            for demo_idx, demo_data in enumerate(data):
                print(f"Processing demonstration {demo_idx}")
                
                # Process demonstration data
                demo_array = self.process_gait_demo(demo_data)
                
                if demo_array is not None and len(demo_array) > 0:
                    demonstrations.append(demo_array)
                    print(f"✓ Demo {demo_idx}: {len(demo_array)} points")
                else:
                    print(f"✗ Demo {demo_idx}: Invalid or empty")
                    
            print(f"Successfully loaded {len(demonstrations)} demonstrations")
            return demonstrations
            
        except Exception as e:
            print(f"✗ Error loading {json_file}: {e}")
            return []
    
    def process_gait_demo(self, demo_data: Dict) -> np.ndarray:
        """
        Process single gait demonstration into TP-GMM format
        
        Args:
            demo_data: Dictionary with demonstration data
            
        Returns:
            Array with TP-GMM data [N x 11] = [time | 5D_FR1 | 5D_FR2]
        """
        try:
            # Extract time (already normalized)
            time_data = np.array(demo_data['time']).flatten()
            n_points = len(time_data)
            
            # Extract FR1 data (robot leg frame)
            pos_fr1 = np.array(demo_data['ankle_pos_FR1'])  # [N x 2]
            vel_fr1 = np.array(demo_data['ankle_pos_FR1_velocity'])  # [N x 2]
            orient_fr1 = np.deg2rad(np.array(demo_data['ankle_orientation_FR1']).flatten())  # [N x 1]
            
            # Extract FR2 data (task frame)
            pos_fr2 = np.array(demo_data['ankle_pos_FR2'])  # [N x 2]
            vel_fr2 = np.array(demo_data['ankle_pos_FR2_velocity'])  # [N x 2]
            orient_fr2 = np.deg2rad(np.array(demo_data['ankle_orientation_FR2']).flatten())  # [N x 1]
            
            # Combine FR1 data: [pos_x, pos_y, vel_x, vel_y, orient]
            fr1_data = np.column_stack([
                pos_fr1,           # 2D position
                vel_fr1,           # 2D velocity  
                orient_fr1         # 1D orientation
            ])
            
            # Combine FR2 data: [pos_x, pos_y, vel_x, vel_y, orient]
            fr2_data = np.column_stack([
                pos_fr2,           # 2D position
                vel_fr2,           # 2D velocity
                orient_fr2         # 1D orientation
            ])
            
            # Create TP-GMM data: [time | FR1 | FR2]
            tpgmm_data = np.column_stack([
                time_data.reshape(-1, 1),  # Time dimension
                fr1_data,                   # 5D FR1
                fr2_data                    # 5D FR2
            ])
            
            print(f"  Processed {n_points} points, shape: {tpgmm_data.shape}")
            return tpgmm_data
            
        except Exception as e:
            print(f"Error processing demonstration: {e}")
            return None
    
    def plot_first_demonstrations(self, demonstrations: List[np.ndarray], n_demos: int = 3):
        """
        Plot first n demonstrations showing trajectories for both frames
        
        Args:
            demonstrations: List of demonstration arrays
            n_demos: Number of demonstrations to plot
        """
        n_demos = min(n_demos, len(demonstrations))
        colors = plt.cm.Set1(np.linspace(0, 1, n_demos))
        
        # Create figure with subplots
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f'First {n_demos} Gait Demonstrations', fontsize=16)
        
        # Column titles
        axes[0, 0].set_title('Frame FR1 (Robot Leg)', fontsize=14, fontweight='bold')
        axes[0, 1].set_title('Frame FR2 (Task Frame)', fontsize=14, fontweight='bold')
        
        for demo_idx in range(n_demos):
            demo = demonstrations[demo_idx]
            time = demo[:, 0]
            color = colors[demo_idx]
            label = f'Demo {demo_idx + 1}'
            
            # FR1 data (columns 1-5)
            pos_fr1 = demo[:, 1:3]      # position x,y
            vel_fr1 = demo[:, 3:5]      # velocity x,y
            orient_fr1 = demo[:, 5]     # orientation
            
            # FR2 data (columns 6-10)
            pos_fr2 = demo[:, 6:8]      # position x,y
            vel_fr2 = demo[:, 8:10]     # velocity x,y
            orient_fr2 = demo[:, 10]    # orientation
            
            # Plot FR1 - Position
            axes[0, 0].plot(pos_fr1[:, 0], pos_fr1[:, 1], 
                           color=color, label=label, linewidth=2)
            axes[0, 0].set_xlabel('Position X')
            axes[0, 0].set_ylabel('Position Y')
            axes[0, 0].grid(True, alpha=0.3)
            axes[0, 0].legend()
            
            # Plot FR2 - Position
            axes[0, 1].plot(pos_fr2[:, 0], pos_fr2[:, 1], 
                           color=color, label=label, linewidth=2)
            axes[0, 1].set_xlabel('Position X')
            axes[0, 1].set_ylabel('Position Y')
            axes[0, 1].grid(True, alpha=0.3)
            axes[0, 1].legend()
            
            # Plot FR1 - Velocity
            axes[1, 0].plot(vel_fr1[:, 0], vel_fr1[:, 1], 
                           color=color, label=label, linewidth=2)
            axes[1, 0].set_xlabel('Velocity X')
            axes[1, 0].set_ylabel('Velocity Y')
            axes[1, 0].grid(True, alpha=0.3)
            axes[1, 0].legend()
            
            # Plot FR2 - Velocity
            axes[1, 1].plot(vel_fr2[:, 0], vel_fr2[:, 1], 
                           color=color, label=label, linewidth=2)
            axes[1, 1].set_xlabel('Velocity X')
            axes[1, 1].set_ylabel('Velocity Y')
            axes[1, 1].grid(True, alpha=0.3)
            axes[1, 1].legend()
            
            # Plot FR1 - Orientation
            axes[2, 0].plot(time, orient_fr1, 
                           color=color, label=label, linewidth=2)
            axes[2, 0].set_xlabel('Time')
            axes[2, 0].set_ylabel('Orientation (rad)')
            axes[2, 0].grid(True, alpha=0.3)
            axes[2, 0].legend()
            
            # Plot FR2 - Orientation
            axes[2, 1].plot(time, orient_fr2, 
                           color=color, label=label, linewidth=2)
            axes[2, 1].set_xlabel('Time')
            axes[2, 1].set_ylabel('Orientation (rad)')
            axes[2, 1].grid(True, alpha=0.3)
            axes[2, 1].legend()
        
        plt.tight_layout()
        plt.savefig(f'plots/gait_demonstrations_{n_demos}.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def compute_and_plot_pca(self, demonstrations: List[np.ndarray]):
        """
        Compute and plot PCA for both frames
        
        Args:
            demonstrations: List of demonstration arrays
        """
        # Combine all demonstrations
        all_data = np.vstack(demonstrations)
        
        # Extract FR1 and FR2 data (without time)
        fr1_data = all_data[:, 1:6]   # 5D FR1 data
        fr2_data = all_data[:, 6:11]  # 5D FR2 data
        
        # Compute PCA
        pca_fr1 = PCA(n_components=2)
        pca_fr2 = PCA(n_components=2)
        
        fr1_transformed = pca_fr1.fit_transform(fr1_data)
        fr2_transformed = pca_fr2.fit_transform(fr2_data)
        
        # Create figure
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))
        fig.suptitle('PCA Analysis of Gait Data', fontsize=16)
        
        # Plot FR1 PCA
        scatter1 = axes[0].scatter(fr1_transformed[:, 0], fr1_transformed[:, 1], 
                                  c=all_data[:, 0], cmap='viridis', alpha=0.6)
        axes[0].set_title(f'FR1 PCA (Explained variance: {pca_fr1.explained_variance_ratio_.sum():.2%})')
        axes[0].set_xlabel(f'PC1 ({pca_fr1.explained_variance_ratio_[0]:.1%})')
        axes[0].set_ylabel(f'PC2 ({pca_fr1.explained_variance_ratio_[1]:.1%})')
        axes[0].grid(True, alpha=0.3)
        plt.colorbar(scatter1, ax=axes[0], label='Time')
        
        # Plot FR2 PCA
        scatter2 = axes[1].scatter(fr2_transformed[:, 0], fr2_transformed[:, 1], 
                                  c=all_data[:, 0], cmap='viridis', alpha=0.6)
        axes[1].set_title(f'FR2 PCA (Explained variance: {pca_fr2.explained_variance_ratio_.sum():.2%})')
        axes[1].set_xlabel(f'PC1 ({pca_fr2.explained_variance_ratio_[0]:.1%})')
        axes[1].set_ylabel(f'PC2 ({pca_fr2.explained_variance_ratio_[1]:.1%})')
        axes[1].grid(True, alpha=0.3)
        plt.colorbar(scatter2, ax=axes[1], label='Time')
        
        plt.tight_layout()
        plt.savefig(f'plots/pca_analysis_{len(demonstrations)}.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # Print PCA components
        print("\n=== PCA Analysis Results ===")
        print(f"FR1 - Explained variance ratio: {pca_fr1.explained_variance_ratio_}")
        print(f"FR1 - Total explained variance: {pca_fr1.explained_variance_ratio_.sum():.2%}")
        print(f"FR2 - Explained variance ratio: {pca_fr2.explained_variance_ratio_}")
        print(f"FR2 - Total explained variance: {pca_fr2.explained_variance_ratio_.sum():.2%}")
        
        return pca_fr1, pca_fr2
    
    def optimize_n_components(self, data: np.ndarray, max_components: int = 15) -> int:
        """
        Optimize number of GMM components using BIC/AIC
        
        Args:
            data: Training data
            max_components: Maximum components to test
            
        Returns:
            Optimal number of components
        """
        n_components_range = range(2, min(max_components, len(data)//10) + 1)
        bic_scores = []
        aic_scores = []
        best_bic = np.inf
        best_n_components = 2
        
        print(f"Optimizing number of components (2 to {max(n_components_range)})...")
        
        for n in n_components_range:
            try:
                gmm = GaussianMixture(
                    n_components=n,
                    covariance_type='full',
                    reg_covar=1e-6,
                    random_state=42,
                    max_iter=100
                )
                gmm.fit(data)
                
                bic = gmm.bic(data)
                aic = gmm.aic(data)
                
                bic_scores.append(bic)
                aic_scores.append(aic)
                
                if bic < best_bic:
                    best_bic = bic
                    best_n_components = n
                    
                print(f"  n={n}: BIC={bic:.1f}, AIC={aic:.1f}")
                    
            except Exception as e:
                print(f"  n={n}: Error - {e}")
                break
        
        print(f"✓ Optimal components: {best_n_components} (BIC: {best_bic:.1f})")
        return best_n_components
    
    def plot_gmm_gaussians(self, gmm_model, data: np.ndarray, feature_indices: Tuple[int, int] = (1, 2)):
        """
        Plot GMM Gaussians in 2D projection
        
        Args:
            gmm_model: Trained GMM model
            data: Training data
            feature_indices: Which features to plot (default: first 2 spatial features)
        """
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Extract 2D features
        x_idx, y_idx = feature_indices
        X = data[:, [x_idx, y_idx]]
        
        # Plot data points colored by time
        scatter = ax.scatter(X[:, 0], X[:, 1], c=data[:, 0], cmap='viridis', alpha=0.6, s=20)
        plt.colorbar(scatter, label='Time')
        
        # Plot Gaussian ellipses
        colors = plt.cm.Set1(np.linspace(0, 1, gmm_model.n_components))
        
        for i in range(gmm_model.n_components):
            # Get mean and covariance for this component
            mean_2d = gmm_model.means_[i, [x_idx, y_idx]]
            cov_2d = gmm_model.covariances_[i][[x_idx, y_idx]][:, [x_idx, y_idx]]
            
            # Compute eigenvalues and eigenvectors
            eigenvals, eigenvecs = np.linalg.eigh(cov_2d)
            
            # Calculate ellipse parameters
            angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
            width = 2 * np.sqrt(eigenvals[0]) * 2  # 2 sigma
            height = 2 * np.sqrt(eigenvals[1]) * 2  # 2 sigma
            
            # Create ellipse
            ellipse = Ellipse(mean_2d, width, height, angle=angle, 
                            facecolor=colors[i], alpha=0.3, 
                            edgecolor=colors[i], linewidth=2,
                            label=f'Component {i+1}')
            ax.add_patch(ellipse)
            
            # Plot component center
            ax.plot(mean_2d[0], mean_2d[1], 'o', color=colors[i], 
                   markersize=8, markeredgecolor='black', markeredgewidth=1)
        
        ax.set_xlabel(f'Feature {x_idx}')
        ax.set_ylabel(f'Feature {y_idx}')
        ax.set_title(f'GMM Components (Features {x_idx} vs {y_idx})')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        feature_name = f"features_{feature_indices[0]}_{feature_indices[1]}"
        plt.tight_layout()
        plt.savefig(f'plots/gmm_advanced_{feature_name}.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def train_tpgmm_model(self, demonstrations: List[np.ndarray]) -> Dict:
        """
        Train TP-GMM model on gait data
        
        Args:
            demonstrations: List of processed demonstrations
            
        Returns:
            Dictionary with trained model and metadata
        """
        print(f"\n=== Training TP-GMM on Gait Data ===")
        print(f"Demonstrations: {len(demonstrations)}")
        
        # Combine all demonstrations
        all_data = np.vstack(demonstrations)
        print(f"Total points: {len(all_data)}")
        print(f"Data dimension: {all_data.shape[1]} (expected: 11 = 1 time + 10 TP-GMM)")
        
        # Optimize number of components
        n_components = self.optimize_n_components(all_data, max_components=20)
        
        # Train final model
        print(f"\nTraining GMM with {n_components} components...")
        gmm = GaussianMixture(
            n_components=n_components,
            covariance_type='full',
            reg_covar=1e-6,
            random_state=42,
            max_iter=300,
            init_params='kmeans'
        )
        
        gmm.fit(all_data)
        
        # Calculate metrics
        log_likelihood = gmm.score(all_data)
        bic = gmm.bic(all_data)
        aic = gmm.aic(all_data)
        
        print(f"✓ Model trained:")
        print(f"  Log-likelihood: {log_likelihood:.2f}")
        print(f"  BIC: {bic:.1f}")
        print(f"  AIC: {aic:.1f}")
        
        # Structure model data
        model_data = {
            'gmm_model': gmm,
            'training_data': all_data,
            'individual_demos': demonstrations,
            'n_components': n_components,
            'metrics': {
                'log_likelihood': log_likelihood,
                'bic': bic,
                'aic': aic
            },
            'data_structure': {
                'total_dim': self.total_dim + 1,  # +1 for time
                'time_dim': 0,
                'fr1_dims': list(range(1, self.point_dim + 1)),
                'fr2_dims': list(range(self.point_dim + 1, self.total_dim + 1)),
                'position_dims': {
                    'fr1': [1, 2],
                    'fr2': [6, 7]
                },
                'velocity_dims': {
                    'fr1': [3, 4],
                    'fr2': [8, 9]
                },
                'orientation_dims': {
                    'fr1': [5],
                    'fr2': [10]
                }
            },
            'frame_info': {
                'num_frames': self.num_frames,
                'dims_per_frame': self.point_dim
            }
        }
        
        return model_data
    
    def save_model(self, model_data: Dict, filename: str):
        """Save trained TP-GMM model"""
        try:
            joblib.dump(model_data, filename)
            print(f"✓ Model saved: {filename}")
            
            # Save readable info
            info_file = filename.replace('.pkl', '_info.txt')
            with open(info_file, 'w') as f:
                f.write("=== TP-GMM Gait Model Info ===\n\n")
                f.write(f"Components: {model_data['n_components']}\n")
                f.write(f"Total dimension: {model_data['data_structure']['total_dim']}\n")
                f.write(f"Demonstrations: {len(model_data['individual_demos'])}\n")
                f.write(f"Training points: {len(model_data['training_data'])}\n\n")
                f.write("Metrics:\n")
                for metric, value in model_data['metrics'].items():
                    f.write(f"  {metric}: {value:.2f}\n")
            
            print(f"✓ Info saved: {info_file}")
            
        except Exception as e:
            print(f"✗ Error saving model: {e}")

def main():
    """
    Main training pipeline for gait TP-GMM
    """
    # Initialize trainer
    trainer = TPGMMGaitTrainer()
    
    # Load gait data
    basepath = 'data/new_processed_gait_data'
    especific_path = '#39_16'
    extension = '.json'
    json_file = f'{basepath}{especific_path}{extension}'

    try:
        print("=== Loading Gait Demonstrations ===")
        demonstrations = trainer.load_gait_demonstrations(json_file)
        
        if len(demonstrations) == 0:
            print("✗ No valid demonstrations found!")
            return
        
        print(f"✓ Loaded {len(demonstrations)} demonstrations")
        
        # Plot first 3 demonstrations
        print("\n=== Plotting Trajectories ===")
        trainer.plot_first_demonstrations(demonstrations, n_demos=10)
        
        # Compute and plot PCA
        print("\n=== Computing PCA ===")
        pca_fr1, pca_fr2 = trainer.compute_and_plot_pca(demonstrations)
        
        # Train TP-GMM model
        print("\n=== Training TP-GMM ===")
        model_data = trainer.train_tpgmm_model(demonstrations)
        
        # Plot GMM Gaussians
        print("\n=== Plotting GMM Components ===")
        # Plot for FR1 position
        trainer.plot_gmm_gaussians(model_data['gmm_model'], 
                                 model_data['training_data'], 
                                 feature_indices=(1, 2))
        
        # Plot for FR2 position  
        trainer.plot_gmm_gaussians(model_data['gmm_model'], 
                                 model_data['training_data'], 
                                 feature_indices=(6, 7))
        
        # Save model
        print("\n=== Saving Model ===")
        trainer.save_model(model_data, f'data/tpgmm_gait_model{especific_path}.pkl')

        print("\n✓ TP-GMM training completed successfully!")
        print(f"  Demonstrations: {len(demonstrations)}")
        print(f"  Components: {model_data['n_components']}")
        print(f"  Data dimension: {model_data['data_structure']['total_dim']}")
        
    except Exception as e:
        print(f"✗ Error in training pipeline: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
