#include "estimateLaneParam.h"

estimateLaneParam::estimateLaneParam(/* args */)
{

}

estimateLaneParam::~estimateLaneParam()
{
}

void estimateLaneParam::setData(const Eigen::VectorXd &matrix_X,float speed,float lookForwardTime,float w,const Eigen::VectorXd &matrix_Z)
{
    speed_           = speed;
    lookForwardTime_ = lookForwardTime;
    matrix_X_        = matrix_X;
    w_               = w;
    matrix_Z_        = matrix_Z;
}

 void estimateLaneParam::estimateLaneLineParam(Eigen::MatrixXd &matrix_P,Eigen::VectorXd &matrix_X)
 {
    matrix_P_ = matrix_P;
    float lookAheadDist = speed_ * lookForwardTime_;
    float dx = lookAheadDist;


    Eigen::MatrixXd matrix_A(4, 4);
    matrix_A.setZero(); // 将矩阵设置为全0
    matrix_A(0, 0) = 1;
    matrix_A(0, 1) = dx;
    matrix_A(0, 2) = pow(dx,2)/2;
    matrix_A(0, 3) = pow(dx,3)/6;

    matrix_A(1, 1) = 1;  
    matrix_A(1, 2) = dx; 
    matrix_A(1, 3) =  pow(dx,2)/2;

    matrix_A(2,2)  = 1;
    matrix_A(2,3)  = dx;

    matrix_A(3,3)  = 1;
    //std::cout << " matrx_A = "<< matrix_A << std::endl;

    Eigen::MatrixXd matrix_B(4, 1);
    matrix_B.setZero();
    matrix_B(0,0) = -pow(dx,2)/(2 * speed_);
    matrix_B(1,0) = -lookForwardTime_;
    //std::cout << "matrx_b = " << matrix_B << std::endl;

    Eigen::MatrixXd matrix_H(4, 4); 
    matrix_H.setZero();
    matrix_H(0,0) = 1;
    matrix_H(1,1) = 1;
    matrix_H(2,2) = 1;
    matrix_H(3,3) = 1;
    //std::cout << "matrix_H = " << matrix_H << std::endl;


    // matrix_P_.resize(4,4);
    // Eigen::VectorXd diag_P(4);
    // diag_P(0) = 0.001;
    // diag_P(1) = 0.001;
    // diag_P(2) = 0.001;
    // diag_P(3) = 0.001;
    // matrix_P_.diagonal() = diag_P; // 将对角线元素分别设置为1.0, 2.0, 3.0
    // std::cout << "matrix_P_ = " << matrix_P_ << std::endl;

    matrix_Q_.resize(4,4);
    Eigen::VectorXd diag_Q(4);
    diag_Q(0) = 0.001;
    diag_Q(1) = 0.001;
    diag_Q(2) = 0.001;
    diag_Q(3) = 0.001;
    matrix_Q_.diagonal() = diag_Q; // 将对角线元素分别设置为1.0, 2.0, 3.0
   // std::cout << "matrix_Q_ = " << matrix_Q_ << std::endl;

    matrix_R_.resize(4,4);
    Eigen::VectorXd diag_R(4);
    diag_R(0) = 0.1;
    diag_R(1) = 0.1;
    diag_R(2) = 0.1;
    diag_R(3) = 0.1;
    matrix_R_.diagonal() = diag_R; // 将对角线元素分别设置为1.0, 2.0, 3.0
    //std::cout << "matrix_R_ = " << matrix_R_ << std::endl;

    // matrix_X_.resize(4);
    // matrix_X_(0) = laneParam_.c0_;
    // matrix_X_(1) = laneParam_.c1_;
    // matrix_X_(2) = laneParam_.c2_;
    // matrix_X_(3) = laneParam_.c3_;

   matrix_U_.resize(1);
   matrix_U_(0) = w_;

    KalmanFilter kalman(matrix_A,matrix_B,matrix_H,matrix_P_,matrix_Q_,matrix_R_,matrix_X_,matrix_U_);

    kalman.Predict();
    kalman.Update(matrix_Z_);

    matrix_X = kalman.x_;
    matrix_P = kalman.P_;
 }
