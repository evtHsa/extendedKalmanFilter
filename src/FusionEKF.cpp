#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "assert.h"

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

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  // the initial transition matrix F_
  // Lesson 25, Unit 14, Laser Measurements, Part 4 (s/kf/ekf)
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  /**
   * TODO: Finish initializing the FusionEKF. <done>
   * TODO: Set the process and measurement noises<done>
   */
  // init H_laser (from H Matrix Quiz, Lesson 25, Unit 11
  // selects just px, py from measurement
    H_laser_ << 1, 0, 0, 0,
                       0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

// https://www.mathsisfun.com/polar-cartesian-coordinates.html
// x = r × cos( θ )
// y = r × sin( θ )

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.<done>
     * TODO: Create the covariance matrix.<done>
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    ekf_.x_ = VectorXd(4);

    assert((measurement_pack.sensor_type_ == MeasurementPackage::LASER) ||
           (measurement_pack.sensor_type_ == MeasurementPackage::RADAR));
        
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coords & init state <done>
      float ro       = measurement_pack.raw_measurements_(0);
      float phi      = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      
      ekf_.x_(0) = ro * cos(phi);
      ekf_.x_(1) = ro * sin(phi);      
      ekf_.x_(2) = ro_dot * cos(phi);
      ekf_.x_(3) = ro_dot * sin(phi);
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
   * TODO: Update the state transition matrix F according to the new elapsed time.<done>
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.<done>
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  // dt units are seconds
  previous_timestamp_ = measurement_pack.timestamp_; // current is the new prev

  float dt_2 = dt   * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;


  // set the acceleration noise components(per todo reqt above)
  float noise_ax = 9;
  float noise_ay = 9;
  
  // set the process covariance matrix Q
  // Lesson 25, Unit 14, Laser Measurements, Part 4 (s/kf/ekf)
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:<done>
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  assert((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) ||
         (measurement_pack.sensor_type_ == MeasurementPackage::LASER));
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates<done>
    // ref: Lesson 25:19(adapted to ekf obj)
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_; // measurement matrix
    ekf_.R_ = R_radar_; // covariance matrix 
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates<done>
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
