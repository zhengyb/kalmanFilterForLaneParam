/*
*@Function:Estimate lne line Param
*@Create by:juchunyu@qq.com
*@Date:2024-12-08 16:00:01
*/
#pragma once
#include<iostream>
#include<vector>
#include <Eigen/Dense>
#include "kalman_filter.h"

using namespace Eigen;

struct laneParamInfo
{
    float c0_;
    float c1_;
    float c2_;
    float c3_;
};

class estimateLaneParam
{
private:
    laneParamInfo laneParam_;
    float     speed_;
    float     lookForwardTime_;
    float     w_;

    Eigen::MatrixXd matrix_P_;
    Eigen::MatrixXd matrix_Q_;
    Eigen::MatrixXd matrix_R_;
    Eigen::VectorXd matrix_Z_;
    Eigen::VectorXd matrix_X_;// state vector
    Eigen::VectorXd matrix_U_;// control vector

  
public:
    estimateLaneParam();
    void setData(const Eigen::VectorXd &matrix_X,float speed,float lookForwardTime,float w,const Eigen::VectorXd &matrix_Z);
    void estimateLaneLineParam(Eigen::MatrixXd &matrix_P,Eigen::VectorXd &matrix_X);
    ~estimateLaneParam();

};

