#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
  int check = 1;
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  // Sigma point dimension
  n_sig_ = 2 * n_aug_ + 1;

  ///* Weights of sigma points
  weights_ = VectorXd(n_sig_);

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* the current NIS for radar
  NIS_radar_ = 0;

  ///* the current NIS for laser
  NIS_laser_ = 0;

  // Sigma points(prediction)
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // IsInitialization
  is_initialized_ = false;

  // Previous time stamp
  previous_timestamp_ = 0;
 
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // Initialization
  // previous_timestamp_ = meas_package.timestamp_;

  if (!is_initialized_)
  {
    int check = 1;
    // Initialize the state x_ with the first measurement
    // Create the covariance matrix
    cout << "Unscented Kalman Filter: " << endl;
    x_ = VectorXd(n_x_);
    x_ << 1, 1, 1, 1, 1;

    P_ = MatrixXd(n_x_, n_x_);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Convert radar from polar to cartesian coordinates and initialize state

      float rho     = meas_package.raw_measurements_(0);
      float phi     = meas_package.raw_measurements_(1);
      float rho_dot = meas_package.raw_measurements_(2);

      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v_ = sqrt(vx * vx + vy * vy);

      x_ << rho * cos(phi),
            rho * sin(phi),
            v_,
            0,
            0;
     }
     else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
     {
       // Initialize state
       x_ << meas_package.raw_measurements_(0),
             meas_package.raw_measurements_(1),
             0,
             0,
             0;
     }

    // Done initializing
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  // Delta t
  float delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;
  
  // Prediction
  Prediction(delta_t);

  // Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    UpdateLidar(meas_package);
  }
  
  cout << "X: " << x_ << endl;
  cout << "P: " << P_ << endl;
  cout << "step: " << check << endl;

  check = check + 1;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //////////////////////////// Calculate Sigma Points ////////////////////////////
  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Augmented sigma point 
  MatrixXd Xsig_aug  = MatrixXd(n_aug_, n_sig_);

  // Calculate augmented mean state 
  x_aug.fill(0);
  x_aug.head(n_x_) = x_;
  
  // Calculate augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  MatrixXd A = P_aug.llt().matrixL();

  // Set sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  //////////////////////////// Predict Sigma Points ////////////////////////////
  for (int i = 0; i < n_sig_; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug(0,i);
    double p_y      = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) 
    {
        px_p = p_x + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else 
    {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //////////////////////////// Mean & Covariance of Predicted Sigma Points ////////////////////////////  

  // Set weights
  weights_.fill(0.5 / (n_aug_ + lambda_));
  float weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;

  // Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sig_; i++)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++)
  {
    //State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalizax_difftion
    x_diff(3) = Normalization(x_diff(3));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  MatrixXd H_ = MatrixXd(2,5);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  MatrixXd R_ = MatrixXd(2,2);
  R_ << std_laspx_ * std_laspx_, 0,
        0, std_laspy_ * std_laspy_;
  
  // Identity matrix 
  MatrixXd I_ = MatrixXd::Identity(n_x_,n_x_);

  VectorXd z = meas_package.raw_measurements_;
  VectorXd y_ = z - H_ * x_;

  MatrixXd H_trans = H_.transpose();
  MatrixXd S_ = H_ * P_ * H_trans + R_; 
  MatrixXd S_inv = S_.inverse();
  MatrixXd K_ = P_ * H_trans * S_inv;

  x_ = x_ + (K_ * y_);

  //angle normalization
  x_(3) = Normalization(x_(3));

  P_ = (I_ - K_ * H_) * P_;

  // NIS Laser
  NIS_laser_ = y_.transpose() * S_inv * y_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //////////////////////////// Predict Radar Measurement ////////////////////////////
  // Number of measurements
  int n_z = 3;
  
  // Create sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) 
  {  
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double vx = cos(yaw) * v;
    double vy = sin(yaw) * v;

    // r, phi, r_dot
    Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);                          
    Zsig(1,i) = atan2(p_y,p_x);                                       
    Zsig(2,i) = (p_x * vx + p_y * vy ) / sqrt(p_x * p_x + p_y * p_y); 
  }

  // Calculate Mean Predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig_; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Calculate Measurement Covariance Matrix S and T
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // Angle normalization
    z_diff(1) = Normalization(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
    
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
  }


  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
      
  S = S + R; 

  //////////////////////////// UKF Update ////////////////////////////
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) 
  {  
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = Normalization(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = Normalization(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  VectorXd z = meas_package.raw_measurements_;

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = Normalization(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

}

float UKF::Normalization(float value) 
{
  //angle normalization
  while (value> M_PI) value -= 2.*M_PI;
  while (value<-M_PI) value += 2.*M_PI;

  return value;
}

