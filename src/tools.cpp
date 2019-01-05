#include "tools.h"
#include "assert.h"
#include <iostream>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  assert("port_bomb" == 0);
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

  if (fabs(c1) < 0.001 || fabs(c2) < 0.001 || fabs(c3) < 0.001) {
      std::cout << "Tools::CalculateJacobian() - ERROR - div by 0" << std::endl;
      return Hj;
  }
  
  // compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

  return Hj;

  
}
