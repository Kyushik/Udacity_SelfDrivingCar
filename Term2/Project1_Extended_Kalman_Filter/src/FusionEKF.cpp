#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // H matrix laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Jacobian Matrix 
  Hj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
        
  // State Covariance Matrix
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);

      ekf_.x_ << rho * cos(phi),
                 rho * sin(phi),
                 rho_dot * cos(phi),
                 rho_dot * sin(phi);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_(0),
                 measurement_pack.raw_measurements_(1),
                 0,
                 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt; 

  float noise_ax = 9;
  float noise_ay = 9;

  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4,4); 
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
             0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ay / 2,
             dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
             0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;
  
  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  
  Tools tools;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.H_ = Hj_;
	  ekf_.R_ = R_radar_;

    VectorXd Z_radar(3);
    Z_radar << measurement_pack.raw_measurements_(0),
               measurement_pack.raw_measurements_(1),
               measurement_pack.raw_measurements_(2);

    ekf_.UpdateEKF(Z_radar);
  } 
  else 
  {
    // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;

    VectorXd Z_lidar(2);
    Z_lidar << measurement_pack.raw_measurements_(0),
               measurement_pack.raw_measurements_(1);

    ekf_.Update(Z_lidar);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
