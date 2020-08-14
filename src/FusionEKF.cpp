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

  ekf_.Init(std::move(Eigen::VectorXd::Zero(4)),      // x_
            std::move(Eigen::MatrixXd::Zero(4,4)),    // P_
            std::move(Eigen::MatrixXd::Identity(4,4)),// F_
            std::move(Eigen::MatrixXd::Zero(3,4)),    // H_
            std::move(Eigen::MatrixXd::Zero(3,3)),    // R_
            std::move(Eigen::MatrixXd::Zero(4,4)));   // Q_

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
      double rho		= measurement_pack.raw_measurements_(0);
      double phi		= measurement_pack.raw_measurements_(1);
      double rhoDot	= measurement_pack.raw_measurements_(2);
      double px = rho * cos(phi);
      double py = rho * cos(phi);
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      double px		= measurement_pack.raw_measurements_(0);
      double py		= measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
    }

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
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
