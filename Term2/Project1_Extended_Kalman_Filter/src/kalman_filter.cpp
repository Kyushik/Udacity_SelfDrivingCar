#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd I_(4,4);
  I_ = MatrixXd::Identity(4, 4);

  VectorXd y_ = z - H_ * x_;

  MatrixXd H_trans = H_.transpose();
  MatrixXd S_ = H_ * P_ * H_trans + R_;
  MatrixXd S_inv = S_.inverse();
  MatrixXd K_ = P_ * H_trans * S_inv;

  x_ = x_ + (K_ * y_);
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd I_(4,4);
  I_ = MatrixXd::Identity(4, 4);

  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(x * x + y * y);
  float phi = 0;

  if (fabs(x) > 0.001)
  {
    phi = atan2(y, x);
  }

  float rho_dot = 0;

  if (fabs(rho) > 0.001)
  {
    rho_dot = (x * vx + y * vy) / rho;
  }

 
  VectorXd h_x(3);
  h_x << rho, phi, rho_dot;

  VectorXd y_ = z - h_x;

  float pi = 3.141592;
  // Normalize angle value in y_ between -pi and pi
  while ((y_(1) > pi) || (y_(1) < -pi))
  {
    if (y_(1) < -pi)
    {
      y_(1) = y_(1) + 2 * pi;
    }

    if (y_(1) > pi)
    {
      y_(1) = y_(1) - 2 * pi;
    }
  }


  MatrixXd H_trans = H_.transpose();
  MatrixXd S_ = H_ * P_ * H_trans + R_;
  MatrixXd S_inv = S_.inverse();
  MatrixXd K_ = P_ * H_trans * S_inv;

  x_ = x_ + K_ * y_;
  P_ = (I_ - K_ * H_) * P_;
}
