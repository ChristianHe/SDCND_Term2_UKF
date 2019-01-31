#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
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
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ = false;

  // State dimension
  n_x_  = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3.0 - n_x_;  

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  //initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Laser measurement function matrix
  H_laser_ = MatrixXd(2, 5);

  // Laser measurement noise matrix
  R_laser_ = MatrixXd(2, 2);

  // Radar measurement noise matrix
  R_radar_ = MatrixXd(3, 3);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   * 1, if unintialized, initialize the x_ P_ H_ R_ first with lidar or radar data.
   * 2, predict process
   * 2.1, generate the sigma point
   * 2.2, generate the aug sigma point
   * 2.3, predict the aug sigma point
   * 2.4, predict the mean and covariance
   * 3, update process
   * 3.1, predict the measurement with lidar data
   * 3.2, update the state
   * 3.3, predict the measurement with radar data
   * 3.4, update the state
   */
  do
  {
    /* 1, Initialize the x_ P_ H_laser_ R_laser_ R_radar_ with first data. */
    if(is_initialized_ == false)
    {
      /* 1.1, Initialize the x_ with lidar or radar data. */
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
      {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        cout << "Initialize KF with first RADAR data" << endl;
        float ro     = meas_package.raw_measurements_(0);
        float theta  = meas_package.raw_measurements_(1);
        float ro_dot = meas_package.raw_measurements_(2);
        float px = ro * cos(theta); //x is verical
        float py = ro * sin(theta); //y is horizon
        float v = ro_dot;
        float yaw = 0;
        float yaw_rate = 0;
        x_ << px, py, v, yaw, yaw_rate;
        //cout << "initial px py: " << px << py << endl;
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
      {
        cout << "Initialize KF with first LASER data." << endl;
        float px = meas_package.raw_measurements_(0);
        float py = meas_package.raw_measurements_(1);
        float v = 0;
        float yaw = 0;
        float yaw_rate = 0;
        x_ << px, py, v, yaw, yaw_rate; 
      }

      /* 1.2, Initialize the P_, H_laser, R_laser, R_radar_. */
      P_ << 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0;

      H_laser_ << 1, 0, 0, 0, 0,
                  0, 1, 0, 0, 0;

      R_laser_ << std_laspx_*std_laspx_, 0,
                  0, std_laspy_*std_laspy_;

      R_radar_ << std_radr_*std_radr_, 0, 0,
                  0 , std_radphi_*std_radphi_, 0,
                  0, 0, std_radrd_*std_radrd_;
      
      time_us_ = meas_package.timestamp_;

      is_initialized_ = true;
      break;
    }

    double dalta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    
    /* 2, predict process */
    Prediction(dalta_t);

    /* 3, update process */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      UpdateRadar(meas_package);
    } 
    else 
    {
      UpdateLidar(meas_package);
    }

    time_us_ = meas_package.timestamp_;

  } while (0);
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  //cout << "delta_t: " << delta_t << endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  //prepare the z_
  float px = meas_package.raw_measurements_(0);
  float py = meas_package.raw_measurements_(1);
  Eigen::VectorXd z_ = VectorXd(2, 1);
  z_ << px, py;

  //cout << "Lidar px py: " << px << py << endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  //prepare the z_
  float ro     = meas_package.raw_measurements_(0);
  float theta  = meas_package.raw_measurements_(1);
  float ro_dot = meas_package.raw_measurements_(2);
  Eigen::VectorXd z_ = VectorXd(3, 1);
  z_ << ro, theta, ro_dot; 

  //cout << "Radar ro theta ro_dot: " << ro << theta << ro_dot << endl;
}