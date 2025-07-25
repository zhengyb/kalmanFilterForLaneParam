"""
Main program for Lane Parameter Estimation using Kalman Filter
Equivalent to the C++ main.cpp
"""
import numpy as np
from estimate_lane_param import EstimateLaneParam, LaneParamInfo


def main():
    """
    Main function demonstrating lane parameter estimation
    """
    # Initialize error covariance matrix P (equivalent to matrix_P in C++)
    matrix_P = np.zeros((4, 4))
    diag_P = np.array([0.001, 0.001, 0.001, 0.001])
    np.fill_diagonal(matrix_P, diag_P)
    print(f"matrix_P_ = \n{matrix_P}")
    
    # Initialize lane parameters (equivalent to laneParam in C++)
    lane_param = LaneParamInfo()
    lane_param.c0_ = 1.8
    lane_param.c1_ = 0.1
    lane_param.c2_ = 0.001
    lane_param.c3_ = 0.000001
    
    # Initialize state vector (equivalent to matrix_X in C++)
    matrix_X = np.array([
        lane_param.c0_,
        lane_param.c1_,
        lane_param.c2_,
        lane_param.c3_
    ])
    
    print(f"matrix_X_init = {matrix_X}")
    
    # Main estimation loop
    for i in range(10):
        estimate_instance = EstimateLaneParam()
        
        # Set parameters
        look_dis_time = 0.5
        speed = 3.6
        w = 0.0
        
        # Create measurement vector (equivalent to matrix_Z in C++)
        matrix_Z = np.array([
            1.95 + 0.3 * i,
            0.13 + 0.01 * i,
            0.006 + 0.001 * i,
            0.000001
        ])
        
        print(f"measurement matrix_Z = {matrix_Z}")
        
        # Set data and estimate
        estimate_instance.set_data(matrix_X, speed, look_dis_time, w, matrix_Z)
        estimate_instance.estimate_lane_line_param(matrix_P, matrix_X)
        
        print(f"matrix_X_res = {matrix_X}")
        print("-" * 50)


if __name__ == "__main__":
    main() 