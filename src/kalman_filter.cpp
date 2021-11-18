#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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

void KalmanFilter::NormalizeAngle(double& phi){
  phi = atan2(sin(phi), cos(phi));
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateCommon(const VectorXd& y){
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H_ * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  // Update the state by using Kalman Filter equations
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd z_pred = VectorXd(3);
  float rho_pred = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi_pred = atan2(x_(1),x_(0));
  float rho_dot_pred = (x_(0)*x_(2)+x_(1)*x_(3))/rho_pred;
  
  z_pred << rho_pred, phi_pred, rho_dot_pred;

  VectorXd y = z - z_pred;
  // ensure that innovation vector y(1) is between -pi..pi
  NormalizeAngle(y(1));
  UpdateCommon(y);
}
