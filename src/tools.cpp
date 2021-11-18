#include "tools.h"
#include <iostream>
#include <algorithm>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    return rmse;
  }
  
  // initialize the residuals vector
  VectorXd residual(4);
  residual << 0,0,0,0;
  VectorXd residual_square(4);
  residual_square << 0,0,0,0;
  VectorXd residual_sum(4);
  residual_sum << 0,0,0,0;
  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    residual = estimations[i] - ground_truth[i];
    residual_square = residual.array() * residual.array();
    residual_sum = residual_sum + residual_square;
    
  }

  // calculate the mean
  VectorXd mean(4);
  mean = residual_sum/estimations.size();

  // calculate the squared root
  rmse = mean.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float px_2 = px*px;
  float py_2 = py*py;
  float c1 = px_2 + py_2;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  

  // check division by zero
  const float eps = 0.001;
  c1 = std::max(eps, c1);
  
  // compute the Jacobian matrix
  
  Hj(0,0) = px/c2;
  Hj(0,1) = py/c2;
  Hj(1,0) = -py/c1;
  Hj(1,1) = px/c1;
  Hj(2,0) = py*(vx*py - vy*px)/c3;
  Hj(2,1) = px*(px*vy - py*vx)/c3;
  Hj(2,2) = px/c2;
  Hj(2,3) = py/c2;

  return Hj;
}
