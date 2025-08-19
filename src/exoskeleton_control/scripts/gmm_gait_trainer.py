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

class GMMGaitTrainer:
    def __init__(self):
        """
        Traditional GMM Trainer for gait data using single frame of reference (FR1)
        
        Frame dimensions:
        - Position: 2D (x, y)
        - Velocity: 2D (x, y) 
        - Orientation: 1D
        Total: 5D
        """
        self.num_frames = 1  # Only FR1
        
        # Dimensions per frame
        self.dims = {
            'position': 2,    # x, y
            'velocity': 2,    # vx, vy
            'orientation': 1  # theta
        }
        self.point_dim = sum(self.dims.values())  # 5D per frame
        self.total_dim = self.point_dim * self.num_frames  # 5D total
        
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
        Process single gait demonstration into GMM format (single frame)
        
        Args:
            demo_data: Dictionary with demonstration data
            
        Returns:
            Array with GMM data [N x 6] = [time | 5D_FR1]
        """
        try:
            # Extract time (already normalized)
            time_data = np.array(demo_data['time']).flatten()
            n_points = len(time_data)
            
            # Extract FR1 data (robot leg frame) only
            pos_fr1 = np.array(demo_data['ankle_pos_FR1'])  # [N x 2]
            vel_fr1 = np.array(demo_data['ankle_pos_FR1_velocity'])  # [N x 2]
            orient_fr1 = np.deg2rad(np.array(demo_data['ankle_orientation_FR1']).flatten())  # [N x 1]
            
            # Combine FR1 data: [pos_x, pos_y, vel_x, vel_y, orient]
            fr1_data = np.column_stack([
                pos_fr1,           # 2D position
                vel_fr1,           # 2D velocity  
                orient_fr1         # 1D orientation
            ])
            
            # Create GMM data: [time | FR1]
            gmm_data = np.column_stack([
                time_data.reshape(-1, 1),  # Time dimension
                fr1_data                   # 5D FR1
            ])
            
            print(f"  Processed {n_points} points, shape: {gmm_data.shape}")
            return gmm_data
            
        except Exception as e:
            print(f"Error processing demonstration: {e}")
            return None
    
    def plot_first_demonstrations(self, demonstrations: List[np.ndarray], n_demos: int = 3):
        """
        Plot first n demonstrations showing trajectories for single frame
        
        Args:
            demonstrations: List of demonstration arrays
            n_demos: Number of demonstrations to plot
        """
        n_demos = min(n_demos, len(demonstrations))
        colors = plt.cm.Set1(np.linspace(0, 1, n_demos))
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f'First {n_demos} Gait Demonstrations - FR1 Frame', fontsize=16)
        
        for demo_idx in range(n_demos):
            demo = demonstrations[demo_idx]
            time = demo[:, 0]
            color = colors[demo_idx]
            label = f'Demo {demo_idx + 1}'
            
            # FR1 data (columns 1-5)
            pos_fr1 = demo[:, 1:3]      # position x,y
            vel_fr1 = demo[:, 3:5]      # velocity x,y
            orient_fr1 = demo[:, 5]     # orientation
            
            # Plot FR1 - Position
            axes[0, 0].plot(pos_fr1[:, 0], pos_fr1[:, 1], 
                           color=color, label=label, linewidth=2)
            axes[0, 0].set_xlabel('Position X (m)')
            axes[0, 0].set_ylabel('Position Y (m)')
            axes[0, 0].set_title('Ankle Position')
            axes[0, 0].grid(True, alpha=0.3)
            axes[0, 0].legend()
            
            # Plot FR1 - Velocity
            axes[0, 1].plot(vel_fr1[:, 0], vel_fr1[:, 1], 
                           color=color, label=label, linewidth=2)
            axes[0, 1].set_xlabel('Velocity X (m/s)')
            axes[0, 1].set_ylabel('Velocity Y (m/s)')
            axes[0, 1].set_title('Ankle Velocity')
            axes[0, 1].grid(True, alpha=0.3)
            axes[0, 1].legend()
            
            # Plot FR1 - Orientation over time
            axes[1, 0].plot(time, orient_fr1, 
                           color=color, label=label, linewidth=2)
            axes[1, 0].set_xlabel('Time (normalized)')
            axes[1, 0].set_ylabel('Orientation (rad)')
            axes[1, 0].set_title('Ankle Orientation')
            axes[1, 0].grid(True, alpha=0.3)
            axes[1, 0].legend()
            
            # Plot Position X over time
            axes[1, 1].plot(time, pos_fr1[:, 0], 
                           color=color, label=label, linewidth=2)
            axes[1, 1].set_xlabel('Time (normalized)')
            axes[1, 1].set_ylabel('Position X (m)')
            axes[1, 1].set_title('Ankle X Position vs Time')
            axes[1, 1].grid(True, alpha=0.3)
            axes[1, 1].legend()
        
        plt.tight_layout(pad=3.0)  # Increase padding to prevent title interference
        os.makedirs('plots', exist_ok=True)
        plt.savefig(f'plots/gmm_gait_demonstrations_{n_demos}.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def compute_and_plot_pca(self, demonstrations: List[np.ndarray]):
        """
        Compute and plot PCA for FR1 frame
        
        Args:
            demonstrations: List of demonstration arrays
        """
        # Combine all demonstrations
        all_data = np.vstack(demonstrations)
        
        # Extract FR1 data (without time)
        fr1_data = all_data[:, 1:6]   # 5D FR1 data
        
        # Compute PCA
        pca_fr1 = PCA(n_components=2)
        fr1_transformed = pca_fr1.fit_transform(fr1_data)
        
        # Create figure
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))
        fig.suptitle('PCA Analysis of Gait Data (FR1 Frame)', fontsize=16)
        
        # Plot FR1 PCA colored by time
        scatter1 = axes[0].scatter(fr1_transformed[:, 0], fr1_transformed[:, 1], 
                                  c=all_data[:, 0], cmap='viridis', alpha=0.6)
        axes[0].set_title(f'FR1 PCA (Explained variance: {pca_fr1.explained_variance_ratio_.sum():.2%})')
        axes[0].set_xlabel(f'PC1 ({pca_fr1.explained_variance_ratio_[0]:.1%})')
        axes[0].set_ylabel(f'PC2 ({pca_fr1.explained_variance_ratio_[1]:.1%})')
        axes[0].set_aspect('equal', adjustable='box')  # Equal scale for x and y axes
        axes[0].grid(True, alpha=0.3)
        plt.colorbar(scatter1, ax=axes[0], label='Time')
        
        # Plot explained variance ratio
        components = range(1, min(6, len(pca_fr1.explained_variance_ratio_) + 1))
        pca_full = PCA()
        pca_full.fit(fr1_data)
        axes[1].bar(components, pca_full.explained_variance_ratio_[:len(components)])
        axes[1].set_xlabel('Principal Component')
        axes[1].set_ylabel('Explained Variance Ratio')
        axes[1].set_title('Explained Variance by Component')
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout(pad=3.0)  # Increase padding to prevent title interference
        os.makedirs('plots', exist_ok=True)
        plt.savefig(f'plots/gmm_pca_analysis_{len(demonstrations)}.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # Print PCA components
        print("\n=== PCA Analysis Results ===")
        print(f"FR1 - Explained variance ratio: {pca_fr1.explained_variance_ratio_}")
        print(f"FR1 - Total explained variance: {pca_fr1.explained_variance_ratio_.sum():.2%}")
        
        return pca_fr1
    
    def optimize_n_components(self, data: np.ndarray, max_components: int = 15) -> Tuple[int, List[float], List[float]]:
        """
        Optimize number of GMM components using BIC/AIC
        
        Args:
            data: Training data
            max_components: Maximum components to test
            
        Returns:
            Tuple of (optimal_components, bic_scores, aic_scores)
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
        
        # Plot BIC/AIC scores
        self.plot_model_selection(list(n_components_range)[:len(bic_scores)], bic_scores, aic_scores, best_n_components)
        
        return best_n_components, bic_scores, aic_scores
    
    def plot_model_selection(self, n_components_list: List[int], bic_scores: List[float], aic_scores: List[float], best_n: int):
        """
        Plot BIC scores for model selection
        
        Args:
            n_components_list: List of number of components tested
            bic_scores: List of BIC scores
            aic_scores: List of AIC scores (kept for compatibility but not plotted)
            best_n: Optimal number of components
        """
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        fig.suptitle('GMM Model Selection: BIC Scores', fontsize=16)
        
        # Plot BIC scores only
        ax.plot(n_components_list, bic_scores, 'b-o', linewidth=2, markersize=8, label='BIC')
        ax.axvline(x=best_n, color='red', linestyle='--', linewidth=2, label=f'Optimal (n={best_n})')
        ax.set_xlabel('Number of Components')
        ax.set_ylabel('BIC Score')
        ax.set_title('Bayesian Information Criterion')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_xticks(n_components_list)
        
        # AIC plot commented out - showing only BIC
        # axes[1].plot(n_components_list, aic_scores, 'g-o', linewidth=2, markersize=8, label='AIC')
        # axes[1].axvline(x=best_n, color='red', linestyle='--', linewidth=2, label=f'Optimal (n={best_n})')
        # axes[1].set_xlabel('Number of Components')
        # axes[1].set_ylabel('AIC Score')
        # axes[1].set_title('Akaike Information Criterion')
        # axes[1].grid(True, alpha=0.3)
        # axes[1].legend()
        # axes[1].set_xticks(n_components_list)
        
        plt.tight_layout(pad=3.0)  # Increase padding to prevent title interference
        os.makedirs('plots', exist_ok=True)
        plt.savefig('plots/gmm_model_selection_bic.png', dpi=300, bbox_inches='tight')
        plt.show()
    
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
        
        feature_names = ['Time', 'Pos X', 'Pos Y', 'Vel X', 'Vel Y', 'Orient']
        ax.set_xlabel(f'{feature_names[x_idx]}')
        ax.set_ylabel(f'{feature_names[y_idx]}')
        ax.set_title(f'GMM Components ({feature_names[x_idx]} vs {feature_names[y_idx]})')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        feature_name = f"features_{feature_indices[0]}_{feature_indices[1]}"
        plt.tight_layout(pad=3.0)  # Increase padding to prevent title interference
        os.makedirs('plots', exist_ok=True)
        plt.savefig(f'plots/gmm_components_{feature_name}.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_trajectories_with_gmm_overlay(self, demonstrations: List[np.ndarray], gmm_model, data: np.ndarray, 
                                         feature_indices: Tuple[int, int], plot_name: str, title: str):
        """
        Plot all demonstration trajectories with GMM components overlaid
        
        Args:
            demonstrations: List of demonstration arrays
            gmm_model: Trained GMM model
            data: Training data
            feature_indices: Which features to plot (e.g., (1,2) for pos_x, pos_y)
            plot_name: Name for saving the plot file
            title: Plot title
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # Extract feature indices
        x_idx, y_idx = feature_indices
        feature_names = ['Time', 'Pos X', 'Pos Y', 'Vel X', 'Vel Y', 'Orient']
        
        # Plot GMM components as ellipses FIRST (so trajectories appear on top)
        X = data[:, [x_idx, y_idx]]
        colors_gmm = plt.cm.Set1(np.linspace(0, 1, gmm_model.n_components))
        
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
                            facecolor=colors_gmm[i], alpha=0.4, 
                            edgecolor=colors_gmm[i], linewidth=3,
                            label=f'GMM {i+1}')  # Show ALL GMM components in legend
            ax.add_patch(ellipse)
            
            # Plot component center
            ax.plot(mean_2d[0], mean_2d[1], 'o', color=colors_gmm[i], 
                   markersize=10, markeredgecolor='black', markeredgewidth=2)
        
        # Plot all demonstration trajectories as thin lines ON TOP of GMM ellipses
        colors_demo = plt.cm.tab10(np.linspace(0, 1, len(demonstrations)))
        
        for demo_idx, demo in enumerate(demonstrations):
            traj_x = demo[:, x_idx]
            traj_y = demo[:, y_idx]
            ax.plot(traj_x, traj_y, color=colors_demo[demo_idx], 
                   linewidth=1.5, alpha=0.7, label=f'Demo {demo_idx+1}')
        
        # Formatting
        ax.set_xlabel(f'{feature_names[x_idx]} (m)' if 'Pos' in feature_names[x_idx] else f'{feature_names[x_idx]} (m/s)')
        ax.set_ylabel(f'{feature_names[y_idx]} (m)' if 'Pos' in feature_names[y_idx] else f'{feature_names[y_idx]} (m/s)')
        ax.set_title(title, fontsize=16)
        ax.grid(True, alpha=0.3)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.set_aspect('equal', adjustable='box')  # Equal scaling
        
        plt.tight_layout(pad=3.0)  # Increase padding to prevent title interference
        os.makedirs('plots', exist_ok=True)
        plt.savefig(f'plots/{plot_name}.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def train_gmm_model(self, demonstrations: List[np.ndarray]) -> Dict:
        """
        Train GMM model on gait data
        
        Args:
            demonstrations: List of processed demonstrations
            
        Returns:
            Dictionary with trained model and metadata
        """
        print(f"\n=== Training GMM on Gait Data ===" )
        print(f"Demonstrations: {len(demonstrations)}")
        
        # Combine all demonstrations
        all_data = np.vstack(demonstrations)
        print(f"Total points: {len(all_data)}")
        print(f"Data dimension: {all_data.shape[1]} (expected: 6 = 1 time + 5 GMM)")
        
        # Optimize number of components
        n_components, bic_scores, aic_scores = self.optimize_n_components(all_data, max_components=20)
        
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
                'position_dims': [1, 2],
                'velocity_dims': [3, 4],
                'orientation_dims': [5]
            },
            'frame_info': {
                'num_frames': self.num_frames,
                'dims_per_frame': self.point_dim
            }
        }
        
        return model_data
    
    def save_model(self, model_data: Dict, filename: str):
        """Save trained GMM model"""
        try:
            joblib.dump(model_data, filename)
            print(f"✓ Model saved: {filename}")
            
            # Save readable info
            info_file = filename.replace('.pkl', '_info.txt')
            with open(info_file, 'w') as f:
                f.write("=== GMM Gait Model Info ===\n\n")
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
    Main training pipeline for gait GMM
    """
    # Initialize trainer
    trainer = GMMGaitTrainer()
    
    # Load gait data
    json_file = 'DEXO-Control-Core/src/exoskeleton_control/data/new_processed_gait_data#35_8_p100.json'

    try:
        print("=== Loading Gait Demonstrations ===")
        demonstrations = trainer.load_gait_demonstrations(json_file)
        
        if len(demonstrations) == 0:
            print("✗ No valid demonstrations found!")
            return
        
        print(f"✓ Loaded {len(demonstrations)} demonstrations")
        
        # Plot all demonstrations
        print("\n=== Plotting Trajectories ===")
        trainer.plot_first_demonstrations(demonstrations, n_demos=len(demonstrations))
        
        # Compute and plot PCA
        print("\n=== Computing PCA ===")
        pca_fr1 = trainer.compute_and_plot_pca(demonstrations)
        
        # Train GMM model
        print("\n=== Training GMM ===")
        model_data = trainer.train_gmm_model(demonstrations)
        
        # Plot GMM Gaussians
        print("\n=== Plotting GMM Components ===")
        # Plot for position space
        trainer.plot_gmm_gaussians(model_data['gmm_model'], 
                                 model_data['training_data'], 
                                 feature_indices=(1, 2))
        
        # Plot for velocity space
        trainer.plot_gmm_gaussians(model_data['gmm_model'], 
                                 model_data['training_data'], 
                                 feature_indices=(3, 4))
        
        # Plot trajectories with GMM overlay - Position
        print("\n=== Plotting Position Trajectories with GMM Overlay ===")
        trainer.plot_trajectories_with_gmm_overlay(
            demonstrations, 
            model_data['gmm_model'], 
            model_data['training_data'],
            feature_indices=(1, 2),  # pos_x, pos_y
            plot_name='trajectories_gmm_overlay_position',
            title='Gait Trajectories with GMM Components - Position Space'
        )
        
        # Plot trajectories with GMM overlay - Velocity
        print("\n=== Plotting Velocity Trajectories with GMM Overlay ===")
        trainer.plot_trajectories_with_gmm_overlay(
            demonstrations, 
            model_data['gmm_model'], 
            model_data['training_data'],
            feature_indices=(3, 4),  # vel_x, vel_y
            plot_name='trajectories_gmm_overlay_velocity',
            title='Gait Trajectories with GMM Components - Velocity Space'
        )
        
        # Save model
        print("\n=== Saving Model ===")
        trainer.save_model(model_data, 'DEXO-Control-Core/src/exoskeleton_control/data/gmm_gait_model.pkl')

        print("\n✓ GMM training completed successfully!")
        print(f"  Demonstrations: {len(demonstrations)}")
        print(f"  Components: {model_data['n_components']}")
        print(f"  Data dimension: {model_data['data_structure']['total_dim']}")
        
    except Exception as e:
        print(f"✗ Error in training pipeline: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()