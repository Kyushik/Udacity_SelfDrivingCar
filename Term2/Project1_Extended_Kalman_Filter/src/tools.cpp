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

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size())
    {
        cout << "Vector size is not equal!" << endl;  
        return rmse;
    }
    
    if (estimations.size() == 0)
    {
        cout << "Estimation vector size is 0!" << endl; 
        return rmse;
    }
    
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
    rmse = rmse/estimations.size();
    
	//calculate the squared root
    rmse = rmse.array().sqrt();
	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
    
	//check division by zero
    if (px == 0 && py == 0)
    {
        px = 0.0001;
    }
	//compute the Jacobian matrix
	float px_2 = px * px;
	float py_2 = py * py;
	float pxpy_3 = (px_2 + py_2) * (px_2 + py_2) * (px_2 + py_2);
	
    Hj << px / sqrt(px_2 + py_2), py / sqrt(px_2 + py_2), 0, 0,
         -py / (px_2 + py_2), px / (px_2 + py_2), 0, 0,
          py * (vx * py - vy * px) / sqrt(pxpy_3), px * (vy * px - vx * py) / sqrt(pxpy_3), px / sqrt(px_2 + py_2) , py / sqrt(px_2 + py_2);  
    
	return Hj;
}
