#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  
  // compute the Jacobian matrix
  if( px != 0. && py!= 0.) {
      float pho = (std::pow(px, 2) + std::pow(py, 2));
      float pho_srqt = std::sqrt(pho);
      float mul_pho = pho * pho_srqt;
      Hj<< px / pho_srqt, py / pho_srqt, 0., 0.,
           -py / pho,  px / pho, 0., 0.,
           py * (vx * py - vy * px) / mul_pho, px * (vy * px - vx * py) / mul_pho, px / pho_srqt, py / pho_srqt;
  }else {
      Hj<< 0,0,0,0,
           0,0,0,0,
           0,0,0,0;
  }

  return Hj;
}

