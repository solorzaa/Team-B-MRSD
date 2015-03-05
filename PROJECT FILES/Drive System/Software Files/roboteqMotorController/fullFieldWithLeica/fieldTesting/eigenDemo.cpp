#include <Eigen/Dense>
#include <iostream>
using namespace std;

Eigen::MatrixXd m = Eigen::MatrixXd::Random(6,6);
m = (m + Eigen::MatrixXd::Constant(6,6,1.2)) * 50;

int main()
{
  cout << "m =" << endl << m << endl;
  Eigen::VectorXd v(6);
  v << 1, 2, 3, 4, 5, 6;
  cout << "m * v =" << endl << m * v << endl;
}
