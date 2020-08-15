#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &&x_in, MatrixXd &&P_in, MatrixXd &&F_in,
                        MatrixXd &&H_in, MatrixXd &&R_in, MatrixXd &&Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

// Kalman filter time update (prediction) function
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  // Predict the state
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

// Kalman filter linear update equations 
void KalmanFilter::UpdateLidar(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // Calculate the innovation
  VectorXd y = z - H_*x_;
  // Calculate the innovation covariance and Kalman coefficient
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht*S.inverse();

  // Update the state
  x_ = x_ + (K * y);
  P_ = P_ - K*H_* P_;
}

// Kalman filter nonlinear update equations which uses Jacobian matrix 
void KalmanFilter::UpdateRadar(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // Store the state information
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  // Transform state to measurement space using h() function
  double rho = std::sqrt(px*px + py*py);
  double phi = std::atan2(py, px);
  double rhoDot;

  // Check whether the denominator is close to zero
  if (fabs(rho) < 0.0001) {
    rhoDot = 0;
  } 
  else {
    rhoDot = (px*vx + py*vy)/rho;
  }

  // Store the predicted measurements
  VectorXd z_pred(3);
  z_pred << rho, phi, rhoDot;

  // Calculate the innovation
  VectorXd y = z - z_pred;

  // Since the angle difference can cause prolem, truncate the rho angle
  while (y(1)> M_PI) y(1)-=2.*M_PI;
  while (y(1)<-M_PI) y(1)+=2.*M_PI;

  // Calculate the innovation covariance and Kalman coefficient
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht*S.inverse();

  // Update the state
  x_ = x_ + (K * y);
  P_ = P_ - K*H_* P_;
}
