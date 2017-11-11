#include "kalman_filter.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
#define PI 3.14159265

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_=F_*x_;
  P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y=z-H_*x_;
  MatrixXd S=H_*P_*H_.transpose()+R_;
  MatrixXd K=P_*H_.transpose()*S.inverse();

  x_=x_+K*y;
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h =VectorXd(3);
  h(0)=sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  h(1)=atan2(x_(1) , x_(0));
  h(2)=(x_(0)*x_(2)+x_(1)*x_(3))/(h(0));
  VectorXd y=z-h;
  if (y(1)<-PI){
    while(y(1)<-PI){
      y(1)+=2*PI;
    }
  }
  else if(y(1)>PI){
    while(y(1)>PI){
      y(1)-=2*PI;
    }

  }

  MatrixXd S=H_*P_*H_.transpose()+R_;
  MatrixXd K=P_*H_.transpose()*S.inverse();

  x_=x_+K*y;
  P_ = (I_ - K * H_) * P_;
}
