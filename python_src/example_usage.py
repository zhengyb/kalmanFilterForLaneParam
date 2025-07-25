"""
Example usage of the modified lane parameter estimation
Demonstrates handling cases with and without measurements
"""
import numpy as np
from estimate_lane_param import EstimateLaneParam


def main():
    # Initialize the estimator
    estimator = EstimateLaneParam()
    
    # Example parameters
    speed = 20.0  # m/s
    look_forward_time = 0.5  # seconds
    w = 0.1  # rad/s (angular velocity)
    
    # Initial state vector [c0, c1, c2, c3]
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    
    # Initial error covariance matrix
    matrix_P = np.eye(4) * 0.1
    
    print("=== Lane Parameter Estimation Example ===\n")
    
    # Case 1: With measurement available
    print("Case 1: Measurement Available")
    print("-" * 30)
    
    # Measurement vector
    matrix_Z = np.array([0.05, 0.12, 0.008, 0.002])
    
    # Set data with measurement
    estimator.set_data(matrix_X, speed, look_forward_time, w, matrix_Z)
    
    # Perform full estimation (predict + update)
    estimator.estimate_lane_line_param(matrix_P, matrix_X)
    
    print(f"Initial state: [0.0, 0.1, 0.01, 0.001]")
    print(f"Measurement: {matrix_Z}")
    print(f"Updated state: {matrix_X}")
    print()
    
    # Case 2: No measurement available
    print("Case 2: No Measurement Available")
    print("-" * 30)
    
    # Reset state for this case
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    
    # Set data without measurement
    estimator.set_data(matrix_X, speed, look_forward_time, w, matrix_Z=None)
    
    # Perform prediction only
    estimator.predict_only(matrix_P, matrix_X)
    
    print(f"Initial state: [0.0, 0.1, 0.01, 0.001]")
    print(f"Predicted state (no measurement): {matrix_X}")
    print()
    
    # Case 3: Explicit control over predict/update
    print("Case 3: Explicit Control")
    print("-" * 30)
    
    # Reset state for this case
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    
    # Set data with measurement
    estimator.set_data(matrix_X, speed, look_forward_time, w, matrix_Z)
    
    # Perform prediction only
    estimator.estimate_lane_line_param(matrix_P, matrix_X, measurement_available=False)
    print(f"After prediction only: {matrix_X}")
    
    # Now perform update with measurement
    estimator.estimate_lane_line_param(matrix_P, matrix_X, measurement_available=True)
    print(f"After prediction + update: {matrix_X}")
    print()
    
    # Case 4: Simulation with missing measurements
    print("Case 4: Simulation with Missing Measurements")
    print("-" * 30)
    
    # Reset state for simulation
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    
    # Simulate multiple time steps
    for step in range(5):
        print(f"Step {step + 1}:")
        
        # Simulate measurement availability (missing every 3rd measurement)
        measurement_available = (step + 1) % 3 != 0
        
        if measurement_available:
            # Generate a measurement
            measurement = matrix_X + np.random.normal(0, 0.01, 4)
            estimator.set_data(matrix_X, speed, look_forward_time, w, measurement)
            estimator.estimate_lane_line_param(matrix_P, matrix_X, measurement_available=True)
            print(f"  Measurement available: {measurement}")
        else:
            # No measurement
            estimator.set_data(matrix_X, speed, look_forward_time, w, matrix_Z=None)
            estimator.predict_only(matrix_P, matrix_X)
            print(f"  No measurement available")
        
        print(f"  State: {matrix_X}")
        print()


if __name__ == "__main__":
    main() 