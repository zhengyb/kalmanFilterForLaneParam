/*
*@Function:kalman filter
*@Create by:juchunyu@qq.com
*@Date:2024-12-08 16:00:01
*/
#pragma once
#include<iostream>
#include<vector>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
public:
    KalmanFilter(const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, const MatrixXd& P, const MatrixXd& Q, const MatrixXd& R,const VectorXd& x,const VectorXd& u);
    
    ~KalmanFilter();
    void Predict();
    void Update(const VectorXd& z);
    void UpdateEKF(const VectorXd& z);

    MatrixXd F_; // state transition matrix
    MatrixXd B_; // control matrix
    MatrixXd H_; // measurement matrix
    MatrixXd P_; // error covariance matrix
    MatrixXd Q_; // process noise covariance matrix
    MatrixXd R_; // measurement noise covariance matrix

    VectorXd x_; // state vector
    VectorXd u_; // control vector
};

