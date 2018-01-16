#pragma once
//#include <vector>
//#include "Eigen-3.3/Eigen/Core"

class State;
class Control;

class MPC {
public:
  Control operator()(State s);
//  MPC();

//  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
//  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

//#endif /* MPC_H */
