#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Check the validity
  // 1. The estimation vector size should not be zero
  // 2. The estimation vector size should be equal with ground truth vector size

  if (estimations.size() != ground_truth.size())
  {
    std::cout << "Vector size is NOT equal!!!" << std::endl;
    return rmse;
  }

  if (estimations.size() == 0)
  {
    std::cout << "Estimation vector size is 0!!!" << std::endl;
    return rmse;
  }

  // Accumulate Squared Residuals
  for(int i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];

    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate Mean
  rmse = rmse / estimations.size();

  // Calculate the Squared Root
  rmse = rmse.array().sqrt();
  
  // Return rmse
  return rmse;
}
