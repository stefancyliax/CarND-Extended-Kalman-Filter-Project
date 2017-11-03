#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
  TODO:
    * predict the state
  */

  // predict equations as in lectures
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // update equations as in lectures
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  KalmanFilter::UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // convert current prediction to cartesian for use in later equation
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = std::sqrt(px * px + py * py);
  float theta = std::atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;

  //handling if px and py are near zero
  if (px < 0.001 && py < 0.001)
    {
    float theta = 0;
    float rho_dot = 0;
    }
 
  // cartesian state vector
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  // measurement update
  VectorXd y = z - z_pred;

  // for the prediction to work, the angle has to be between +Pi and -Pi
  // it can be normalized by adding or substracting 2*Pi
  if (y[1] > M_PI)
  {
    y[1] -= 2 * M_PI;
  }
  if (y[1] < -M_PI)
  {
    y[1] += 2 * M_PI;
  }

  // Debug output
  //std::cout << z[1] << "  " << theta << "  y: " << y[1] << std::endl;
  KalmanFilter::UpdateCommon(y);

}

void KalmanFilter::UpdateCommon(const VectorXd &y)
{
  // use common measurement update for linear and EKF approach to keep it DRY
  // measurement update as in lectures
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate equations as in lectures
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}