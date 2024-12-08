#include "estimateLaneParam.h"
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


int main()
{
    Eigen::MatrixXd matrix_P(4,4);
    Eigen::VectorXd diag_P(4);
    diag_P(0) = 0.001;
    diag_P(1) = 0.001;
    diag_P(2) = 0.001;
    diag_P(3) = 0.001;
    matrix_P.diagonal() = diag_P; // 将对角线元素分别设置为1.0, 2.0, 3.0
    std::cout << "matrix_P_ = " << matrix_P << std::endl;

    laneParamInfo laneParam;
    laneParam.c0_ = 1.8;
    laneParam.c1_ = 0.1;
    laneParam.c2_ = 0.001;
    laneParam.c3_ = 0.000001;

    VectorXd matrix_X(4); // state vector
    matrix_X(0) = laneParam.c0_;
    matrix_X(1) = laneParam.c1_;
    matrix_X(2) = laneParam.c2_;
    matrix_X(3) = laneParam.c3_;

    std::cout << "matrix_X_init = " << matrix_X << std::endl;
   
   int i = 0;
    while(i < 10)
    {
        estimateLaneParam estimateInstance;
        float lookDisTime = 0.5;
        float speed = 3.6;
        float w = 0.0;
        Eigen::VectorXd matrix_Z(4); //messare
        matrix_Z(0) = 1.95+0.3*i;
        matrix_Z(1) = 0.13 + 0.01*i;
        matrix_Z(2) = 0.006 + 0.001*i;
        matrix_Z(3) =  0.000001;
        std::cout << "meaasurement matrix_Z = " << matrix_Z << std::endl;
        estimateInstance.setData(matrix_X,speed,lookDisTime,w,matrix_Z);
    
        estimateInstance.estimateLaneLineParam(matrix_P,matrix_X);

        std::cout << "matrix_X_res = " << matrix_X << std::endl;
        i++;
    }
}