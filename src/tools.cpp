#include "tools.h"
#include "assert.h"
#include <iostream>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.<done>
   */
  // per Lesson 25, Unit 23 Evaluating KF Performance Part 1 calculate rmse
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0){
      std::cout << "Invalid estimation or ground_truth data" << std::endl;
      return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i){

      VectorXd residual = estimations[i] - ground_truth[i];

      //coefficient-wise multiplication
      residual = residual.array() * residual.array();
      rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
 return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:<done>
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
    
  // get position and velocity from state
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // per Lesson 25, Unit 20: Jacobian Matrix, Part 2, precompute freq used terms
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  if (fabs(c1) < epsilon || fabs(c2) < epsilon || fabs(c3) < epsilon) {
      std::cout << "Tools::CalculateJacobian() - ERROR - div by 0" << std::endl;
      return Hj;
  }
  
  // compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

  return Hj;

  
}
