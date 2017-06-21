#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
    // Initialize the errors
    p_error = 0;
    i_error = 0;
    d_error = 0;

    // Initialize the coefficients
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;

}

void PID::UpdateCoefficient(double Kp, double Ki, double Kd)
{
    // Update PID coefficients
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;
}

void PID::UpdateError(double cte) 
{
    // Update PID error
    double cte_old = p_error; 

    p_error = cte;
    i_error += cte;
    d_error = cte - cte_old;

}

double PID::TotalError() 
{
    // Calculate Steering angle 
    std::cout << "P value: " << -Kp * p_error << std::endl;
    std::cout << "I value: " << -Ki * i_error << std::endl;
    std::cout << "D value: " << -Kd * d_error << std::endl;

    double steering_angle = -Kp * p_error -Ki * i_error -Kd * d_error;
    return steering_angle;
}

