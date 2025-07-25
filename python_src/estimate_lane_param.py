"""
Lane Parameter Estimation using Kalman Filter
Equivalent to the C++ estimateLaneParam.h and estimateLaneParam.cpp
"""
import numpy as np
from kalman_filter import KalmanFilter


class LaneParamInfo:
    """
    Lane parameter information structure
    Equivalent to the C++ laneParamInfo struct
    """
    def __init__(self, c0=0.0, c1=0.0, c2=0.0, c3=0.0):
        self.c0_ = c0
        self.c1_ = c1
        self.c2_ = c2
        self.c3_ = c3


class EstimateLaneParam:
    """
    Lane parameter estimation using Kalman Filter
    Equivalent to the C++ estimateLaneParam class
    """
    
    def __init__(self):
        """
        Initialize the lane parameter estimator
        """
        self.lane_param_ = LaneParamInfo()
        self.speed_ = 0.0
        self.look_forward_time_ = 0.0
        self.w_ = 0.0
        
        # Matrices for Kalman Filter
        self.matrix_P_ = None
        self.matrix_Q_ = None
        self.matrix_R_ = None
        self.matrix_Z_ = None
        self.matrix_X_ = None  # state vector
        self.matrix_U_ = None  # control vector
        
        # Kalman Filter instance
        self.kalman_ = None
    
    def set_motion_data(self, speed, look_forward_time, w):
        """
        Set motion data for prediction
        
        Args:
            speed: vehicle speed
            look_forward_time: look forward time
            w: angular velocity
        """
        self.speed_ = speed
        self.look_forward_time_ = look_forward_time
        self.w_ = w
    
    def set_state_data(self, matrix_X, matrix_P):
        """
        Set state data for estimation
        
        Args:
            matrix_X: state vector
            matrix_P: error covariance matrix
        """
        self.matrix_X_ = matrix_X.copy()
        self.matrix_P_ = matrix_P.copy()
    
    def set_measurement_data(self, matrix_Z):
        """
        Set measurement data for update
        
        Args:
            matrix_Z: measurement vector
        """
        self.matrix_Z_ = matrix_Z.copy()
    
    def _initialize_matrices(self):
        """
        Initialize Kalman Filter matrices
        """
        # Calculate look ahead distance
        look_ahead_dist = self.speed_ * self.look_forward_time_
        dx = look_ahead_dist
        
        # State transition matrix A (equivalent to matrix_A in C++)
        matrix_A = np.zeros((4, 4))
        matrix_A[0, 0] = 1
        matrix_A[0, 1] = dx
        matrix_A[0, 2] = pow(dx, 2) / 2
        matrix_A[0, 3] = pow(dx, 3) / 6
        
        matrix_A[1, 1] = 1
        matrix_A[1, 2] = dx
        matrix_A[1, 3] = pow(dx, 2) / 2
        
        matrix_A[2, 2] = 1
        matrix_A[2, 3] = dx
        
        matrix_A[3, 3] = 1
        
        # Control matrix B (equivalent to matrix_B in C++)
        matrix_B = np.zeros((4, 1))
        matrix_B[0, 0] = -pow(dx, 2) / (2 * self.speed_)
        matrix_B[1, 0] = -self.look_forward_time_
        
        # Measurement matrix H (equivalent to matrix_H in C++)
        matrix_H = np.zeros((4, 4))
        matrix_H[0, 0] = 1
        matrix_H[1, 1] = 1
        matrix_H[2, 2] = 1
        matrix_H[3, 3] = 1
        
        # Process noise covariance matrix Q
        self.matrix_Q_ = np.zeros((4, 4))
        diag_Q = np.array([0.001, 0.001, 0.001, 0.001])
        np.fill_diagonal(self.matrix_Q_, diag_Q)
        
        # Measurement noise covariance matrix R； 固定测量噪声；
        self.matrix_R_ = np.zeros((4, 4))
        diag_R = np.array([0.1, 0.1, 0.1, 0.1])
        np.fill_diagonal(self.matrix_R_, diag_R)
        
        # Control vector U
        self.matrix_U_ = np.array([[self.w_]])
        
        # Create Kalman Filter instance
        self.kalman_ = KalmanFilter(
            matrix_A, matrix_B, matrix_H, 
            self.matrix_P_, self.matrix_Q_, self.matrix_R_,
            self.matrix_X_, self.matrix_U_
        )
    
    def predict(self, matrix_P, matrix_X):
        """
        Perform prediction step only
        
        Args:
            matrix_P: error covariance matrix (input/output)
            matrix_X: state vector (input/output)
        """
        # Set current state
        self.set_state_data(matrix_X, matrix_P)
        
        # Initialize matrices if needed
        if self.kalman_ is None:
            self._initialize_matrices()
        else:
            # Update Kalman Filter with current state
            self.kalman_.x_ = self.matrix_X_.copy()
            self.kalman_.P_ = self.matrix_P_.copy()
        
        # Perform prediction
        self.kalman_.predict()
        
        # Update output parameters
        matrix_X[:] = self.kalman_.x_[:4]
        matrix_P[:] = self.kalman_.P_[:]

    def update(self, matrix_P, matrix_X, matrix_Z):
        """
        Perform update step only
        """
        self.set_measurement_data(matrix_Z)
        self.set_state_data(matrix_X, matrix_P)
        # Update Kalman Filter with current state
        self.kalman_.x_ = self.matrix_X_.copy()
        self.kalman_.P_ = self.matrix_P_.copy()        
        self.kalman_.update(self.matrix_Z_)
        # Update output parameters
        matrix_X[:] = self.kalman_.x_[:4]
        matrix_P[:] = self.kalman_.P_[:]
    
    def predict_and_update(self, matrix_P, matrix_X, matrix_Z):
        """
        Perform both prediction and update steps in sequence
        
        Args:
            matrix_P: error covariance matrix (input/output)
            matrix_X: state vector (input/output)
            matrix_Z: measurement vector
        """
        # First predict
        self.predict(matrix_P, matrix_X)
        
        # Then update with measurement
        self.set_measurement_data(matrix_Z)
        
        # Set current state (after prediction)
        self.set_state_data(matrix_X, matrix_P)
        
        # Update Kalman Filter with current state
        self.kalman_.x_ = self.matrix_X_.copy()
        self.kalman_.P_ = self.matrix_P_.copy()
        
        # Perform update
        self.kalman_.update(self.matrix_Z_)
        
        # Update output parameters
        matrix_X[:] = self.kalman_.x_[:4]
        matrix_P[:] = self.kalman_.P_[:]
    
    def predict_only(self, matrix_P, matrix_X):
        """
        Legacy method for backward compatibility
        Perform prediction step only
        """
        self.predict(matrix_P, matrix_X)
    
    # Legacy methods for backward compatibility
    def set_data(self, matrix_X, speed, look_forward_time, w, matrix_Z=None):
        """
        Legacy method for backward compatibility
        Set all data at once
        
        Args:
            matrix_X: state vector
            speed: vehicle speed
            look_forward_time: look forward time
            w: angular velocity
            matrix_Z: measurement vector (optional)
        """
        self.set_motion_data(speed, look_forward_time, w)
        # For legacy compatibility, we don't set P here as it's passed to estimate_lane_line_param
        self.matrix_X_ = matrix_X.copy()
        if matrix_Z is not None:
            self.set_measurement_data(matrix_Z)
    
    def estimate_lane_line_param(self, matrix_P, matrix_X, measurement_available=True):
        """
        Legacy method for backward compatibility
        Estimate lane line parameters using Kalman Filter
        
        Args:
            matrix_P: error covariance matrix (input/output)
            matrix_X: state vector (input/output)
            measurement_available: whether measurement is available for update step
        """
        if measurement_available and self.matrix_Z_ is not None:
            self.predict_and_update(matrix_P, matrix_X, self.matrix_Z_)
        else:
            self.predict(matrix_P, matrix_X) 