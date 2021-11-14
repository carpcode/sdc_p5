#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
<<<<<<< HEAD
  /**
   * TODO: Calculate the RMSE here.
   */
=======
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
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
>>>>>>> bb5d5f42f6aabe00124a17d189488dca961f989b
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
