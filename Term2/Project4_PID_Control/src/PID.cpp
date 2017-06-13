#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
    // Initialize the errors
    p_error = 0;
    i_error = 0;
    d_error = 0;

    // Initializa the coefficients
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    sum_cte = 0;
    sum_error = 0;
}

void PID::UpdateError(double cte, double cte_old, double dt) 
{
    sum_cte = sum_cte + cte * dt;

    p_error = cte;
    i_error = sum_cte;
    d_error = (cte - cte_old) / dt;
    std::cout<<d_error<<std::endl;
}

double PID::TotalError() 
{
    double steering_angle = -Kp * p_error -Ki * i_error -Kd * d_error;
    return steering_angle;
}

