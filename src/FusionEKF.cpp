#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  Hj_ = Eigen::MatrixXd::Zero(3,4);          

  ekf_.Init(std::move(Eigen::VectorXd::Ones(4)),      // x_
            std::move(Eigen::MatrixXd::Zero(4,4)),    // P_
            std::move(Eigen::MatrixXd::Identity(4,4)),// F_
            std::move(Eigen::MatrixXd::Zero(3,4)),    // H_
            std::move(Eigen::MatrixXd::Zero(3,3)),    // R_
            std::move(Eigen::MatrixXd::Zero(4,4)));   // Q_
  ekf_.P_ << 5, 0, 0, 0,
             0, 5, 0, 0,
             0, 0, 20, 0,
             0, 0, 0, 20;
  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       *  TODO: Convert radar from polar to cartesian coordinates 
       *        and initialize state.
       */
      // First radar measurement is used to initialize the position of the object.
      // Although radar provides the velocity measurement we do not have yaw angle to convert it to velocity information.
      double rho		= measurement_pack.raw_measurements_(0);
      double phi		= measurement_pack.raw_measurements_(1);
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // First lidar measurements are used to initialize the position of the object.
      double px		= measurement_pack.raw_measurements_(0);
      double py		= measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
    }

    // Initialize the timestamp.
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Calculate the delta-t to predict the state.
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify F so that it contains the delta-t
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Modify Q so that it contains the delta-t
  ekf_.Q_(0,0) = std::pow(dt,4)*noise_ax/4;
  ekf_.Q_(0,2) = std::pow(dt,3)*noise_ax/2;
  ekf_.Q_(1,1) = std::pow(dt,4)*noise_ay/4;
  ekf_.Q_(1,3) = std::pow(dt,3)*noise_ay/2;
  ekf_.Q_(2,0) = std::pow(dt,3)*noise_ax/2;
  ekf_.Q_(2,2) = std::pow(dt,2)*noise_ax;
  ekf_.Q_(3,1) = std::pow(dt,3)*noise_ay/2;
  ekf_.Q_(3,3) = std::pow(dt,2)*noise_ay;

  // EKF prediction
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  // If the sensor type is RADAR then use the UpdateRadar function which uses Jacobian matrix
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
	  ekf_.R_ = R_radar_;
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.UpdateRadar(measurement_pack.raw_measurements_);
  } else {   // If the sensor type is LIDAR then use the UpdateLidar function which uses linear equations
    // TODO: Laser updates
	  ekf_.R_ = R_laser_;
	  ekf_.H_ = H_laser_;
	  ekf_.UpdateLidar(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
