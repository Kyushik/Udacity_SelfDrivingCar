#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  n_x_ = x_.size();

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* the current NIS for radar
  NIS_radar_ = 0;

  ///* the current NIS for laser
  NIS_laser_ = 0;

  // Sigma points(prediction)
  X_sig_pred = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
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

  float previous_timestamp_ = meas_package.timestamp_;

  if (!is_initialized_)
  {
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

      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
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
    is_initialized_ = true;
    return;
  }

  double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;

  Prediction(delta_t);

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
  MatrixXd X_sig_aug  = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  MatrixXd X_sig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1); 

  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Aumented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

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
  X_sig_aug.col(0) = x_aug.col(0);
  for (int i = 0; i < n_aug_; i++)
  {
    X_sig_aug.col(i+1)        = x_ + sqrt(lambda_ + n_aug_) * A.col(i);
    X_sig_aug.col(i+1+n_aug_) = x_ - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  //////////////////////////// Predict Sigma Points ////////////////////////////
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //Extract values for better readability
    double p_x      = X_sig_aug(0,i);
    double p_y      = X_sig_aug(1,i);
    double v        = X_sig_aug(2,i);
    double yaw      = X_sig_aug(3,i);
    double yawd     = X_sig_aug(4,i);
    double nu_a     = X_sig_aug(5,i);
    double nu_yawdd = X_sig_aug(6,i);

    // Predicted state values
    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else 
    {
      px_p = p_x * v * delta_t * cos(yaw);
      py_p = p_y * v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p  = v_p + nu_a * delta_t;

    yaw_p  = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // Write predicted sigma point into right column
    X_sig_pred(0,i) = px_p;
    X_sig_pred(1,i) = py_p;
    X_sig_pred(2,i) = v_p;
    X_sig_pred(3,i) = yaw_p;
    X_sig_pred(4,i) = yawd_p;
  }

  //////////////////////////// Mean & Covariance of Predicted Sigma Points ////////////////////////////  
  // Vector for Weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  // Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ = x_ + weights_(i) * X_sig_pred.col(i);
  }

  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //State difference
    VectorXd x_diff = X_sig_pred.col(i) - x_;
    // Angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  
}

void UKF::PredictLidarMeasurement(VectorXd z_out, MatrixXd S_out, MatrixXd T_out)
{
  //////////////////////////// Predict Lidar Measurement ////////////////////////////
  // Number of measurements
  int n_z_lidar = 2;
  
  // Create sigma points in measurement space
  MatrixXd Z_sig = MatrixXd(n_z_lidar, 2 * n_aug_ + 1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar, n_z_lidar);

  // Cross-correlation matrix
  MatrixXd T = MatrixXd(n_x_, n_z_lidar);

  // Transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    Z_sig(0,i) = X_sig_pred(0,i);
    Z_sig(1,i) = X_sig_pred(1,i);    
  }

  // Calculate Mean Predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Z_sig.col(i);
  }

  // Calculate Measurement Covariance Matrix S and T
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    
    // Angle normalization
    while (z_diff(3) >  M_PI) z_diff(3) -= 2. * M_PI;
    while (z_diff(3) < -M_PI) z_diff(3) += 2. * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
    
    VectorXd x_diff = X_sig_pred.col(i) - x_;

    // Angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    
    T = T + weights_(i) * x_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z_lidar, n_z_lidar);
  R.fill(0.0);

  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;

  S = S + R;

  z_out = z_pred;
  S_out = S;
  T_out = T; 

  return;
}

void UKF::PredictRadarMeasurement(VectorXd z_out, MatrixXd S_out, MatrixXd T_out)
{
  //////////////////////////// Predict Radar Measurement ////////////////////////////
  // Number of measurements
  int n_z_radar = 3;
  
  // Create sigma points in measurement space
  MatrixXd Z_sig = MatrixXd(n_z_radar, 2 * n_aug_ + 1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar, n_z_radar);

  // Cross-correlation matrix
  MatrixXd T = MatrixXd(n_x_, n_z_radar);

  // Transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    float px  = X_sig_pred(0,i);
    float py  = X_sig_pred(1,i);
    float v   = X_sig_pred(2,i);
    float yaw = X_sig_pred(3,i);
    float pxpy_2 = px*px + py*py;

    Z_sig(0,i) = sqrt(pxpy_2);
    Z_sig(1,i) = atan(py / px);
    Z_sig(2,i) = (px * cos(yaw) * v + py * sin(yaw) * v) / sqrt(pxpy_2);    
  }

  // Calculate Mean Predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Z_sig.col(i);
  }

  // Calculate Measurement Covariance Matrix S and T
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    
    // Angle normalization
    while (z_diff(3) >  M_PI) z_diff(3) -= 2. * M_PI;
    while (z_diff(3) < -M_PI) z_diff(3) += 2. * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
    
    VectorXd x_diff = X_sig_pred.col(i) - x_;

    // Angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
    
    T = T + weights_(i) * x_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z_radar, n_z_radar);
  R.fill(0.0);

  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
      
  S = S + R; 

  z_out = z_pred;
  S_out = S;
  T_out = T; 
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
}
