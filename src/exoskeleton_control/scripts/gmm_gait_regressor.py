import numpy as np
import matplotlib.pyplot as plt
import joblib
from typing import Dict, Tuple
import os

class GMMGaitRegressor:
    def __init__(self):
        """
        GMM-based Gaussian Mixture Regression for gait trajectory reproduction
        
        Uses phase (time) as input to predict position and velocity in Cartesian coordinates
        """
        self.model_data = None
        self.gmm_model = None
        
    def load_gmm_model(self, model_path: str) -> bool:
        """
        Load trained GMM model from pickle file
        
        Args:
            model_path: Path to the saved GMM model (.pkl file)
            
        Returns:
            True if loaded successfully, False otherwise
        """
        try:
            print(f"Loading GMM model from: {model_path}")
            self.model_data = joblib.load(model_path)
            self.gmm_model = self.model_data['gmm_model']
            
            print(f"✓ Model loaded successfully!")
            print(f"  Components: {self.model_data['n_components']}")
            print(f"  Data dimension: {self.model_data['data_structure']['total_dim']}")
            print(f"  Training demonstrations: {len(self.model_data['individual_demos'])}")
            
            return True
            
        except Exception as e:
            print(f"✗ Error loading model: {e}")
            return False
    
    def gmr(self, input_data: np.ndarray, input_dims: list, output_dims: list) -> Tuple[np.ndarray, np.ndarray]:
        """
        Perform Gaussian Mixture Regression (GMR)
        
        Args:
            input_data: Input values [N x input_dim]
            input_dims: Indices of input dimensions
            output_dims: Indices of output dimensions
            
        Returns:
            Tuple of (predicted_output, predicted_variance)
        """
        if self.gmm_model is None:
            raise ValueError("No GMM model loaded. Call load_gmm_model() first.")
        
        n_points = input_data.shape[0]
        n_outputs = len(output_dims)
        n_components = self.gmm_model.n_components
        
        # Initialize output arrays
        predicted_output = np.zeros((n_points, n_outputs))
        predicted_variance = np.zeros((n_points, n_outputs, n_outputs))
        
        for i, input_point in enumerate(input_data):
            # Reshape input point for prediction
            input_point = input_point.reshape(1, -1)
            
            # Get responsibilities (posterior probabilities) for each component
            responsibilities = self._compute_responsibilities(input_point, input_dims)
            
            # Compute conditional mean and covariance for each component
            component_means = np.zeros((n_components, n_outputs))
            component_covs = np.zeros((n_components, n_outputs, n_outputs))
            
            for k in range(n_components):
                mean_k = self.gmm_model.means_[k]
                cov_k = self.gmm_model.covariances_[k]
                
                # Partition mean and covariance
                mu_input = mean_k[input_dims]
                mu_output = mean_k[output_dims]
                
                sigma_ii = cov_k[np.ix_(input_dims, input_dims)]
                sigma_oo = cov_k[np.ix_(output_dims, output_dims)]
                sigma_io = cov_k[np.ix_(input_dims, output_dims)]
                sigma_oi = cov_k[np.ix_(output_dims, input_dims)]
                
                # Compute conditional distribution parameters
                try:
                    sigma_ii_inv = np.linalg.inv(sigma_ii + np.eye(len(input_dims)) * 1e-6)
                    
                    # Conditional mean: mu_o + Sigma_oi * Sigma_ii^-1 * (x - mu_i)
                    conditional_mean = mu_output + sigma_oi @ sigma_ii_inv @ (input_point.flatten() - mu_input)
                    
                    # Conditional covariance: Sigma_oo - Sigma_oi * Sigma_ii^-1 * Sigma_io
                    conditional_cov = sigma_oo - sigma_oi @ sigma_ii_inv @ sigma_io
                    
                    component_means[k] = conditional_mean
                    component_covs[k] = conditional_cov
                    
                except np.linalg.LinAlgError:
                    # Handle singular matrix - use pseudoinverse
                    sigma_ii_pinv = np.linalg.pinv(sigma_ii)
                    conditional_mean = mu_output + sigma_oi @ sigma_ii_pinv @ (input_point.flatten() - mu_input)
                    conditional_cov = sigma_oo - sigma_oi @ sigma_ii_pinv @ sigma_io
                    
                    component_means[k] = conditional_mean
                    component_covs[k] = conditional_cov
            
            # Compute final prediction as weighted sum of component predictions
            predicted_output[i] = np.sum(responsibilities[:, np.newaxis] * component_means, axis=0)
            
            # Compute prediction variance
            weighted_cov = np.zeros((n_outputs, n_outputs))
            for k in range(n_components):
                diff = component_means[k] - predicted_output[i]
                weighted_cov += responsibilities[k] * (component_covs[k] + np.outer(diff, diff))
            
            predicted_variance[i] = weighted_cov
        
        return predicted_output, predicted_variance
    
    def _compute_responsibilities(self, input_point: np.ndarray, input_dims: list) -> np.ndarray:
        """
        Compute posterior probabilities (responsibilities) for input point
        
        Args:
            input_point: Input point [1 x input_dim]
            input_dims: Indices of input dimensions
            
        Returns:
            Responsibilities for each component [n_components]
        """
        n_components = self.gmm_model.n_components
        log_probs = np.zeros(n_components)
        
        for k in range(n_components):
            mean_k = self.gmm_model.means_[k, input_dims]
            cov_k = self.gmm_model.covariances_[k][np.ix_(input_dims, input_dims)]
            weight_k = self.gmm_model.weights_[k]
            
            # Compute log probability
            try:
                cov_inv = np.linalg.inv(cov_k + np.eye(len(input_dims)) * 1e-6)
                cov_det = np.linalg.det(cov_k + np.eye(len(input_dims)) * 1e-6)
                
                diff = input_point.flatten() - mean_k
                log_prob = np.log(weight_k) - 0.5 * len(input_dims) * np.log(2 * np.pi) - 0.5 * np.log(cov_det) - 0.5 * diff.T @ cov_inv @ diff
                log_probs[k] = log_prob
                
            except np.linalg.LinAlgError:
                # Handle singular matrix
                log_probs[k] = -np.inf
        
        # Convert to probabilities and normalize
        max_log_prob = np.max(log_probs)
        probs = np.exp(log_probs - max_log_prob)
        responsibilities = probs / (np.sum(probs) + 1e-10)
        
        return responsibilities
    
    def predict_trajectory(self, phase_values: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict position and velocity trajectories from phase values using GMR
        
        Args:
            phase_values: Phase (time) values [N x 1]
            
        Returns:
            Tuple of (positions, position_std, velocities, velocity_std)
        """
        if self.model_data is None:
            raise ValueError("No model loaded. Call load_gmm_model() first.")
        
        # Define dimension indices based on model structure
        input_dims = [0]  # Time/phase dimension
        position_output_dims = [1, 2]  # Position X, Y
        velocity_output_dims = [3, 4]  # Velocity X, Y
        
        print(f"Performing GMR for {len(phase_values)} phase points...")
        
        # Predict positions
        positions, pos_variance = self.gmr(phase_values, input_dims, position_output_dims)
        pos_std = np.sqrt(np.diagonal(pos_variance, axis1=1, axis2=2))
        
        # Predict velocities
        velocities, vel_variance = self.gmr(phase_values, input_dims, velocity_output_dims)
        vel_std = np.sqrt(np.diagonal(vel_variance, axis1=1, axis2=2))
        
        print(f"✓ GMR completed")
        print(f"  Position range: X=[{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}], Y=[{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}]")
        print(f"  Velocity range: X=[{velocities[:, 0].min():.3f}, {velocities[:, 0].max():.3f}], Y=[{velocities[:, 1].min():.3f}, {velocities[:, 1].max():.3f}]")
        
        return positions, pos_std, velocities, vel_std
    
    def plot_recovered_trajectories(self, phase_values: np.ndarray, positions: np.ndarray, pos_std: np.ndarray, 
                                  velocities: np.ndarray, vel_std: np.ndarray):
        """
        Plot recovered position and velocity trajectories with uncertainty bounds
        
        Args:
            phase_values: Phase values used for prediction
            positions: Predicted positions [N x 2]
            pos_std: Position standard deviations [N x 2]
            velocities: Predicted velocities [N x 2] 
            vel_std: Velocity standard deviations [N x 2]
        """
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('GMR Trajectory Recovery Results', fontsize=16)
        
        # Plot original demonstrations for comparison
        if self.model_data and 'individual_demos' in self.model_data:
            demos = self.model_data['individual_demos']
            colors_demo = plt.cm.tab10(np.linspace(0, 1, len(demos)))
            
            for demo_idx, demo in enumerate(demos):
                demo_pos = demo[:, 1:3]  # Position columns
                demo_vel = demo[:, 3:5]  # Velocity columns
                
                # Plot original positions
                axes[0, 0].plot(demo_pos[:, 0], demo_pos[:, 1], 
                               color=colors_demo[demo_idx], alpha=0.3, linewidth=1,
                               label=f'Demo {demo_idx+1}' if demo_idx < 3 else "")
                
                # Plot original velocities
                axes[0, 1].plot(demo_vel[:, 0], demo_vel[:, 1], 
                               color=colors_demo[demo_idx], alpha=0.3, linewidth=1,
                               label=f'Demo {demo_idx+1}' if demo_idx < 3 else "")
        
        # Plot recovered position trajectory
        axes[0, 0].plot(positions[:, 0], positions[:, 1], 'r-', linewidth=3, 
                       label='GMR Mean', zorder=10)
        
        # Add uncertainty bounds for position
        axes[0, 0].fill_between(positions[:, 0], 
                               positions[:, 1] - 2*pos_std[:, 1],
                               positions[:, 1] + 2*pos_std[:, 1],
                               alpha=0.2, color='red', label='±2σ bounds')
        
        axes[0, 0].set_xlabel('Position X (m)')
        axes[0, 0].set_ylabel('Position Y (m)')
        axes[0, 0].set_title('Recovered Position Trajectory')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 0].set_aspect('equal', adjustable='box')
        
        # Plot recovered velocity trajectory
        axes[0, 1].plot(velocities[:, 0], velocities[:, 1], 'b-', linewidth=3, 
                       label='GMR Mean', zorder=10)
        
        # Add uncertainty bounds for velocity
        axes[0, 1].fill_between(velocities[:, 0], 
                               velocities[:, 1] - 2*vel_std[:, 1],
                               velocities[:, 1] + 2*vel_std[:, 1],
                               alpha=0.2, color='blue', label='±2σ bounds')
        
        axes[0, 1].set_xlabel('Velocity X (m/s)')
        axes[0, 1].set_ylabel('Velocity Y (m/s)')
        axes[0, 1].set_title('Recovered Velocity Trajectory')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()
        axes[0, 1].set_aspect('equal', adjustable='box')
        
        # Plot position components vs time
        axes[1, 0].plot(phase_values, positions[:, 0], 'r-', linewidth=2, label='Pos X')
        axes[1, 0].fill_between(phase_values.flatten(), 
                               positions[:, 0] - 2*pos_std[:, 0],
                               positions[:, 0] + 2*pos_std[:, 0],
                               alpha=0.2, color='red')
        
        axes[1, 0].plot(phase_values, positions[:, 1], 'g-', linewidth=2, label='Pos Y')
        axes[1, 0].fill_between(phase_values.flatten(), 
                               positions[:, 1] - 2*pos_std[:, 1],
                               positions[:, 1] + 2*pos_std[:, 1],
                               alpha=0.2, color='green')
        
        axes[1, 0].set_xlabel('Phase (normalized time)')
        axes[1, 0].set_ylabel('Position (m)')
        axes[1, 0].set_title('Position Components vs Phase')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()
        
        # Plot velocity components vs time
        axes[1, 1].plot(phase_values, velocities[:, 0], 'b-', linewidth=2, label='Vel X')
        axes[1, 1].fill_between(phase_values.flatten(), 
                               velocities[:, 0] - 2*vel_std[:, 0],
                               velocities[:, 0] + 2*vel_std[:, 0],
                               alpha=0.2, color='blue')
        
        axes[1, 1].plot(phase_values, velocities[:, 1], 'c-', linewidth=2, label='Vel Y')
        axes[1, 1].fill_between(phase_values.flatten(), 
                               velocities[:, 1] - 2*vel_std[:, 1],
                               velocities[:, 1] + 2*vel_std[:, 1],
                               alpha=0.2, color='cyan')
        
        axes[1, 1].set_xlabel('Phase (normalized time)')
        axes[1, 1].set_ylabel('Velocity (m/s)')
        axes[1, 1].set_title('Velocity Components vs Phase')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()
        
        plt.tight_layout(pad=3.0)
        os.makedirs('plots', exist_ok=True)
        plt.savefig('plots/gmr_recovered_trajectories.png', dpi=300, bbox_inches='tight')
        plt.show()

def main():
    """
    Main function to perform GMR on trained GMM model
    """
    # Initialize regressor
    regressor = GMMGaitRegressor()
    
    # Load the trained model
    model_path = 'DEXO-Control-Core/src/exoskeleton_control/data/gmm_gait_model.pkl'
    
    try:
        print("=== Loading Trained GMM Model ===")
        success = regressor.load_gmm_model(model_path)
        
        if not success:
            print("✗ Failed to load model!")
            return
        
        # Generate phase values for prediction (full gait cycle)
        print("\n=== Generating Phase Values ===")
        n_points = 100  # Number of points for prediction
        phase_values = np.linspace(0, 1, n_points).reshape(-1, 1)
        print(f"Generated {n_points} phase points from 0 to 1")
        
        # Perform GMR to recover trajectories
        print("\n=== Performing GMR ===")
        positions, pos_std, velocities, vel_std = regressor.predict_trajectory(phase_values)
        
        # Plot results
        print("\n=== Plotting Results ===")
        regressor.plot_recovered_trajectories(phase_values, positions, pos_std, velocities, vel_std)
        
        print("\n✓ GMR trajectory recovery completed successfully!")
        print(f"  Predicted points: {len(positions)}")
        print(f"  Phase range: [{phase_values.min():.2f}, {phase_values.max():.2f}]")
        
    except Exception as e:
        print(f"✗ Error in GMR pipeline: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()