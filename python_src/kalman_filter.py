"""
Kalman Filter implementation in Python
Equivalent to the C++ kalman_filter.h and kalman_filter.cpp
"""
import numpy as np


class KalmanFilter:
    """
    Standard Kalman Filter implementation
    """
    
    def __init__(self, A, B, H, P, Q, R, x, u):
        """
        Initialize Kalman Filter
        
        Args:
            A: state transition matrix (F_ in C++)
            B: control matrix
            H: measurement matrix
            P: error covariance matrix
            Q: process noise covariance matrix
            R: measurement noise covariance matrix
            x: state vector
            u: control vector
        """
        self.F_ = A  # state transition matrix
        self.B_ = B  # control matrix
        self.H_ = H  # measurement matrix
        self.P_ = P  # error covariance matrix
        self.Q_ = Q  # process noise covariance matrix
        self.R_ = R  # measurement noise covariance matrix
        self.x_ = x  # state vector
        self.u_ = u  # control vector
    
    def predict(self):
        """
        Predict step of Kalman Filter
        """
        # State prediction: x = F*x + B*u
        # Ensure x_ is a column vector for matrix operations
        if self.x_.ndim == 1:
            x_col = self.x_.reshape(-1, 1)
        else:
            x_col = self.x_
            
        result = self.F_ @ x_col + self.B_ @ self.u_
        self.x_ = result.flatten()  # Convert back to 1D array
        print(f"predict x_: {self.x_}")
        
        # Covariance prediction: P = F*P*F^T + Q
        F_transpose = self.F_.T
        self.P_ = self.F_ @ self.P_ @ F_transpose + self.Q_
    
    def update(self, z):
        """
        Update step of Kalman Filter
        
        Args:
            z: measurement vector
        """
        # Innovation: y = z - H*x
        y = z - self.H_ @ self.x_
        
        # Innovation covariance: S = H*P*H^T + R
        H_transpose = self.H_.T
        S = self.H_ @ self.P_ @ H_transpose + self.R_
        
        # Kalman gain: K = P*H^T*S^(-1)
        S_inv = np.linalg.inv(S)
        K = self.P_ @ H_transpose @ S_inv
        
        # State update: x = x + K*y
        # Ensure y is a column vector for matrix operations
        if y.ndim == 1:
            y_col = y.reshape(-1, 1)
        else:
            y_col = y
            
        update_term = K @ y_col
        self.x_ = self.x_ + update_term.flatten()
        
        # Covariance update: P = (I - K*H)*P
        x_size = self.x_.shape[0]  # Use shape[0] instead of size
        I = np.eye(x_size)
        self.P_ = (I - K @ self.H_) @ self.P_
    
    def update_ekf(self, z):
        """
        Extended Kalman Filter update (placeholder for future implementation)
        """
        # TODO: implement Extended Kalman Filter equations
        pass 