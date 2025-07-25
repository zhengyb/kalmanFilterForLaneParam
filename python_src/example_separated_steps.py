"""
Example usage of the separated predict and update steps
Demonstrates the two realistic scenarios: predict only and predict+update
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
    
    print("=== Separated Predict and Update Steps Example ===\n")
    
    # Set motion data (only need to set once if motion parameters don't change)
    estimator.set_motion_data(speed, look_forward_time, w)
    
    # Case 1: Prediction only (when measurement is missing)
    print("Case 1: Prediction Only (Measurement Missing)")
    print("-" * 40)
    
    print(f"Initial state: {matrix_X}")
    estimator.predict(matrix_P, matrix_X)
    print(f"After prediction: {matrix_X}")
    print()
    
    # Case 2: Predict and Update in sequence (when measurement is available)
    print("Case 2: Predict and Update (Measurement Available)")
    print("-" * 40)
    
    # Reset state
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    matrix_P = np.eye(4) * 0.1
    
    print(f"Initial state: {matrix_X}")
    
    # Generate a measurement
    measurement = np.array([0.05, 0.12, 0.008, 0.002])
    print(f"Measurement: {measurement}")
    
    # Perform predict and update in sequence
    estimator.predict_and_update(matrix_P, matrix_X, measurement)
    print(f"After predict_and_update: {matrix_X}")
    print()
    
    # Case 3: Simulation with missing measurements
    print("Case 3: Simulation with Missing Measurements")
    print("-" * 40)
    
    # Reset state
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    matrix_P = np.eye(4) * 0.1
    
    # Simulate multiple time steps
    for step in range(5):
        print(f"Step {step + 1}:")
        
        # Simulate measurement availability (missing every 3rd measurement)
        measurement_available = (step + 1) % 3 != 0
        
        if measurement_available:
            # Generate a measurement
            measurement = matrix_X + np.random.normal(0, 0.01, 4)
            print(f"  Measurement available: {measurement}")
            
            # Perform predict and update
            estimator.predict_and_update(matrix_P, matrix_X, measurement)
            print(f"  After predict_and_update: {matrix_X}")
        else:
            print(f"  No measurement available")
            
            # Perform prediction only
            estimator.predict(matrix_P, matrix_X)
            print(f"  After prediction only: {matrix_X}")
        
        print()
    
    # Case 4: Legacy methods still work
    print("Case 4: Legacy Methods (Backward Compatibility)")
    print("-" * 40)
    
    # Reset state
    matrix_X = np.array([0.0, 0.1, 0.01, 0.001])
    matrix_P = np.eye(4) * 0.1
    
    # Use legacy set_data method
    measurement = np.array([0.05, 0.12, 0.008, 0.002])
    estimator.set_data(matrix_X, speed, look_forward_time, w, measurement)
    
    # Use legacy estimate_lane_line_param method
    estimator.estimate_lane_line_param(matrix_P, matrix_X, measurement_available=True)
    print(f"Using legacy methods: {matrix_X}")
    print()


if __name__ == "__main__":
    main() 