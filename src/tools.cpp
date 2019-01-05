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
  assert("port bomb" == 0);
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  assert("port bomb" == 0);
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
