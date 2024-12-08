#include"kalman_filter.h"

KalmanFilter::~KalmanFilter(){}

KalmanFilter::KalmanFilter(const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, const MatrixXd& P, const MatrixXd& Q, const MatrixXd& R,const VectorXd& x,const VectorXd& u)
: F_(A), B_(B), H_(H), P_(P), Q_(Q), R_(R), x_(x),u_(u) {}

void KalmanFilter::Predict() {
    x_ = F_ * x_ + B_ * u_;
    std::cout << "predict x_:" << x_ << std::endl; 
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd& z) {
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    x_ = x_ + K * y;
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd& z) {
    // TODO: update the state by using Extended Kalman Filter equations
}

