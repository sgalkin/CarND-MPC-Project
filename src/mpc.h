#pragma once

#include "model.h"
#include "control.h"

class MPC {
public:
  Control operator()(Model s);
//  MPC();

//  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
//  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
