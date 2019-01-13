#include "kalman_filter.h"
#include "assert.h"
#include "tools.h" // just for epsilon definition

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in; // state(x, y, vx, vy)
    P_ = P_in; // obj covariance matrix
    F_ = F_in; // state transition matrix
    H_ = H_in; // measurement matrix
    R_ = R_in; // measurement covariance matrix
    Q_ = Q_in; // process covariance matrix
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state<done>
   */
    // Lesson 25, Unit 14: Laser Measurments, Part
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations<done>
   */

  // ref: lesson 25, unit 7, kalman filter equations in c++, part 1
  // update state
  //assert("FIXME:factor out common code" == NULL);
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // update estimate
   x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations<done>
   */
  // ref: Lesson 25, Unit 14, Laser Measurements, Part 4
  // cartesian -> polar
  //assert("FIXME:be consistent rho .vs. ro" == NULL)
  float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  if (fabs(rho) < epsilon) { // worry about underflow
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  VectorXd z_prev(3);
  // FIXME: more common code
  z_prev << rho, phi, rho_dot; // so we can use vector subtraction
  VectorXd y = z - z_prev;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // updated estimate
  // ref: lesson 25, unit 7, kalman filter equations in c++, part 1
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
