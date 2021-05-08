#ifndef _MYKALMAN_H
#define _MYKALMAN_H
#endif
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cluster/obstaclePose.h>
#include <cluster/obstaclePoseArray.h>



class KalmanFilter
{
private:
    int stateSize; //state variable's dimenssion  
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;

    Eigen::VectorXd x_;

    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P; //coveriance
    Eigen::MatrixXd H; //
    Eigen::MatrixXd R; //measurement noise covariance
    Eigen::MatrixXd Q; //process noise covariance
    
    ros::Subscriber sub_pose_;
    std::ofstream out("home/pzc/rob_cat_ws/src/kalman/out/obstacle_pose.txt",std::ios::app);
    
public:
    KalmanFilter(int stateSize_, int measSize_,int uSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    Eigen::VectorXd update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas,Eigen::VectorXd z_);
    void posecallback();
};


KalmanFilter::KalmanFilter(int stateSize_ = 0, int measSize_ = 0, int uSize_=0) :stateSize(stateSize_), measSize(measSize_), uSize(uSize_, ros::NodeHandle &nh)
{
    
    sub_pose_ = nh.subcriber("obstacle_pose",1 &KalmanFilter::posecallback,this);
    if (stateSize == 0 || measSize == 0)
    {
        std::cerr << "Error, State size and measurement size must bigger than 0\n";
    }

    x.resize(stateSize);
    x.setZero();

    x_.resize(stateSize);
    x_.setZero();

    A.resize(stateSize, stateSize);
    A.setIdentity();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();
}

void KalmanFilter::init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_)
{
    x = x_;
    P = P_;
    R = R_;
    Q = Q_;
}
Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_)
{
    A = A_;
    B = B_;
    u = u_;
    x = A*x + B*u;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q;
    return x;
}

Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_)
{
    A = A_;
    x = A*x;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q; 
//  cout << "P-=" << P<< endl;
    return x;
}

Eigen::VectorXd KalmanFilter::update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas,Eigen::VectorXd z_)
{
    H = H_;
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
    temp1 = H*P*Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P*Ht*temp2;
    z = H*x;
    x_ = x + K*(z_ - z);
    x = x + K*(z_meas-z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K*H)*P;
//  cout << "P=" << P << endl;
    return x_;
}



