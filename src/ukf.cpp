#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

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
  n_x_ = 5;

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

  // Radar predicted measurement vector
  z_pred_ = VectorXd(3);

  // Radar measurement covariance matrix
  S_radar_ = MatrixXd(3,3);

  // Radar cross correlation matrix
  Tc_ = MatrixXd(n_x_, 3);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
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
    if (is_initialized_ == false)
    {
      /* 1.1, Initialize the x_ with lidar or radar data. */
      /*****************************************************
       * Initial value of x_ and P is quite important, because 
       * when the x_ is equal with the ground truth, the Kalman filter
       * is the best estimator, refer to Invariants and Derivatation  
       * chapter of the Kalman filter wiki for more details. 
       *****************************************************/
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
      {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
        cout << "Initialize KF with first RADAR data" << endl;
        float ro = meas_package.raw_measurements_(0);
        float theta = meas_package.raw_measurements_(1);
        float ro_dot = meas_package.raw_measurements_(2);
        float px = ro * cos(theta); //x is verical
        float py = ro * sin(theta); //y is horizon
        float v = ro_dot; // NOTE: ro_dot is different from v. But if initial with the radar data, the rmse will be better.
        float yaw = 0;
        float yaw_rate = 0;
        x_ << px, py, v, yaw, yaw_rate;

        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 10, 0,
              0, 0, 0, 0, 10;

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

        /* laser have high accuracy of px and py, therefore covariance of px and py 
           is small (equal with measurement covariance?), while v yaw and yaw_rate have high intial covariance. */
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 10, 0, 0,
              0, 0, 0, 10, 0,
              0, 0, 0, 0, 10;
      }

      /* 1.2, Initialize the H_laser, R_laser, R_radar_. */
      H_laser_ << 1, 0, 0, 0, 0,
                  0, 1, 0, 0, 0;

      R_laser_ << std_laspx_ * std_laspx_, 0,
                  0, std_laspy_ * std_laspy_;

      R_radar_ << std_radr_ * std_radr_, 0, 0,
                  0, std_radphi_ * std_radphi_, 0,
                  0, 0, std_radrd_ * std_radrd_;

      time_us_ = meas_package.timestamp_;

      // set weights
      double weight_0 = lambda_/(lambda_+n_aug_);
      weights_(0) = weight_0;
      for (int i=1; i<2*n_aug_+1; ++i) 
      {  
        // 2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
      }

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
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      UpdateLidar(meas_package);
    }

    time_us_ = meas_package.timestamp_;

    // print the output
    /*cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;*/

  } while (0);
}

void UKF::Prediction(double delta_t)
{
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  //cout << "delta_t: " << delta_t << endl;

  /*
   * 2.1, generate the aug sigma points
   * 2.2, predict the aug sigma point
   * 2.3, predict the mean and covariance
  */

  /* 2.1, generate the aug sigma points */
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);

  /* 2.2, predict the aug sigma point */
  SigmaPointPrediction(Xsig_aug, delta_t, &Xsig_pred_);

  /* 2.3, predict the mean and covariance */
  PredictMeanAndCovariance(Xsig_pred_, &x_, &P_);

}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  /* prepare the z_ */
  float px = meas_package.raw_measurements_(0);
  float py = meas_package.raw_measurements_(1);
  Eigen::VectorXd z_ = VectorXd(2, 1);
  z_ << px, py;

  /* update the state by using Kalman Filter equations */
  VectorXd y_ = z_ - H_laser_ * x_;
  MatrixXd H_laser_t_ = H_laser_.transpose();
  MatrixXd S_ = H_laser_ * P_ * H_laser_t_ + R_laser_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ = P_ * H_laser_t_ * Si_;
  MatrixXd I;
  I = MatrixXd::Identity(5, 5);

  // new state
  x_ = x_ + (K_ * y_);
  P_ = (I - K_ * H_laser_) * P_;

  //cout << "Lidar px py: " << px << py << endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  //prepare the z_
  float ro = meas_package.raw_measurements_(0);
  float theta = meas_package.raw_measurements_(1);
  float ro_dot = meas_package.raw_measurements_(2);
  Eigen::VectorXd z_ = VectorXd(3, 1);
  z_ << ro, theta, ro_dot;

  //predict the radar measurement z_pred_ and measurement covariance S_
  PredictRadarMeasurement(&z_pred_, &S_radar_);

  //update
  MatrixXd S_radar_i_ = S_radar_.inverse();

  //Kalman gain
  MatrixXd K_ = Tc_ * S_radar_i_;

  //residual
  VectorXd z_diff = z_ - z_pred_;
  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  x_ = x_ + K_ * z_diff;
  P_ = P_ - K_ * S_radar_ * K_.transpose();
}

void UKF::GenerateSigmaPoints(MatrixXd *Xsig_out)
{
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  // calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  // set first column of sigma point matrix
  Xsig.col(0) = x_;

  // set remaining sigma points
  for (int i = 0; i < n_x_; ++i)
  {
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  // print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  // write result
  *Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out)
{

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // print result
  /*std::cout << "Xsig_aug = " << std::endl
            << Xsig_aug << std::endl;*/

  // write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t, MatrixXd* Xsig_out) 
{

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  // print result
  /*std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;*/

  // write result
  *Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(MatrixXd Xsig_pred, VectorXd* x_out, MatrixXd* P_out) 
{
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {  
    // iterate over sigma points
    x = x + weights_(i) * Xsig_pred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  // print result
  /*std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;*/

  // write result
  *x_out = x;
  *P_out = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) 
{
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S = S + R_radar_;

  // cross corelation matrix Tc
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) 
  {
    // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc_ = Tc_ + weights_(i) * x_diff * z_diff.transpose();
  }

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}
