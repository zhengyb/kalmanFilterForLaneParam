"""
Test script to compare Python implementation with expected results
"""
import numpy as np
from estimate_lane_param import EstimateLaneParam, LaneParamInfo


def test_single_iteration():
    """
    Test a single iteration of the lane parameter estimation
    """
    print("Testing single iteration of lane parameter estimation...")
    
    # Initialize matrices (same as C++ version)
    matrix_P = np.zeros((4, 4))
    diag_P = np.array([0.001, 0.001, 0.001, 0.001])
    np.fill_diagonal(matrix_P, diag_P)
    
    # Initialize state vector
    matrix_X = np.array([1.8, 0.1, 0.001, 0.000001])
    
    # Create measurement
    matrix_Z = np.array([1.95, 0.13, 0.006, 0.000001])
    
    # Create estimator and run
    estimator = EstimateLaneParam()
    estimator.set_data(matrix_X, 3.6, 0.5, 0.0, matrix_Z)
    estimator.estimate_lane_line_param(matrix_P, matrix_X)
    
    print(f"Initial state: [1.8, 0.1, 0.001, 0.000001]")
    print(f"Measurement: {matrix_Z}")
    print(f"Final state: {matrix_X}")
    print(f"State change: {matrix_X - np.array([1.8, 0.1, 0.001, 0.000001])}")
    
    return matrix_X


def test_parameter_convergence():
    """
    Test parameter convergence over multiple iterations
    """
    print("\nTesting parameter convergence over 10 iterations...")
    
    # Initialize
    matrix_P = np.zeros((4, 4))
    diag_P = np.array([0.001, 0.001, 0.001, 0.001])
    np.fill_diagonal(matrix_P, diag_P)
    
    matrix_X = np.array([1.8, 0.1, 0.001, 0.000001])
    
    # Track parameter evolution
    c0_history = [matrix_X[0]]
    c1_history = [matrix_X[1]]
    c2_history = [matrix_X[2]]
    c3_history = [matrix_X[3]]
    
    for i in range(10):
        matrix_Z = np.array([
            1.95 + 0.3 * i,
            0.13 + 0.01 * i,
            0.006 + 0.001 * i,
            0.000001
        ])
        
        estimator = EstimateLaneParam()
        estimator.set_data(matrix_X, 3.6, 0.5, 0.0, matrix_Z)
        estimator.estimate_lane_line_param(matrix_P, matrix_X)
        
        c0_history.append(matrix_X[0])
        c1_history.append(matrix_X[1])
        c2_history.append(matrix_X[2])
        c3_history.append(matrix_X[3])
    
    print(f"Parameter evolution:")
    print(f"c0 (lateral offset): {c0_history}")
    print(f"c1 (heading angle): {c1_history}")
    print(f"c2 (curvature): {c2_history}")
    print(f"c3 (curvature rate): {c3_history}")
    
    return c0_history, c1_history, c2_history, c3_history


if __name__ == "__main__":
    # Run tests
    test_single_iteration()
    test_parameter_convergence()
    
    print("\nPython implementation test completed successfully!") 