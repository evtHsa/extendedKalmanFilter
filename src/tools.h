#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include <iostream>

#define epsilon 0.0001 // for underflow protection

inline float
normalize_angle(float a) { // per "tips and tricks" in assignment
    float tmp = a;
    if (a  > M_PI)
        while (a > M_PI)
            a -= 2 * M_PI;
    if (a  < M_PI)
        while (a > M_PI)
            a += 2 * M_PI;
    if (a != tmp)
        std::cout << "normalize_angle: " << tmp << " => " << a << std::endl;
    return a;
}

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};

#endif  // TOOLS_H_
