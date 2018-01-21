#include "mpc.h"

#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <cppad/cppad.hpp>

#include "model.h"
#include "control.h"
#include "polynomial.h"
#include "solve.h"

namespace {
constexpr size_t degree{3};

template<typename Scalar>
using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    
using Dvector = Vector<double>;
using ADvector = Vector<CppAD::AD<double>>;

constexpr size_t nx = 1;
constexpr size_t ng = 0;  
 
class Cost {
  typename ADvector::value_type operator()(const ADvector& x) const {
    assert(nx == x.size());
    return x[0]*x[0] - x[0];
  }
};

class Constraint {
  Dvector lower_bound() const {
    return Dvector::Constant(ng, -std::numeric_limits<Dvector::value_type>::infinity());
  }

  Dvector upper_bound() const {
    return Dvector::Constant(ng, std::numeric_limits<Dvector::value_type>::infinity());
  }

  ADvector operator()(const ADvector& x) const {
    assert(nx == x.size());
    return ADvector(nx);
  }
};

class Variable {
  Dvector lower_bound() const {
    return Dvector::Constant(nx, -std::numeric_limits<Dvector::value_type>::infinity());
  }

  Dvector upper_bound() const {
    return Dvector::Constant(nx, std::numeric_limits<Dvector::value_type>::infinity());
  }

  operator Dvector() const {
    return Dvector::Zero(nx);
  }
};
}

Control MPC::operator()(Model m) {
  auto poly = Polynomial<degree>(std::move(m.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(State::X) = Eigen::VectorXd::LinSpaced(10, -5, 50);
  waypoints.col(State::Y) = poly(waypoints.col(State::X));
/*
  size_t Vars = 6;
  size_t Actuators = 2;
  
  auto solution = solve(Objective(N*Vars + (N - 1)*Actuators, N*Vars, std::move(poly)));
  if(solution.status != decltype(solution)::success) {
    std::cerr << "Failed to find solution for step " << m.count() << "\n";
  }
  std::cout << "Cost " << solution.obj_value << "\n";
*/    
  return Control(0, 0, Eigen::MatrixXd(0, 2), waypoints);
}

/*

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
*/
