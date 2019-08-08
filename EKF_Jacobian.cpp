#include <iostream>
#include <vector>
#include "Dense"

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

  float len_20 = px*px+py*py;
  // check division by zero
  if(len_20<0.0001)
  {
    cout<<"CalculateJacobian () - Error - Division by Zero"<<endl;
  }
  else
  {

    float len = sqrt(len_20);
    float len_15 = len_20*len;
    // compute the Jacobian matrix
    Hj(0,0)= px/len;
    Hj(0,1)= py/len;
    Hj(1,0)= -py/len_20;
    Hj(1,1)= px/len_20;
    Hj(2,0)= py*(vx*py-vy*px)/len_15;
    Hj(2,1)= px*(vy*px-vx*py)/len_15;
    Hj(2,2)= px/len;
    Hj(2,3)= py/len;
  }

  return Hj;
}