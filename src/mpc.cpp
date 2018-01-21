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

class Objective {
public:
  using Dvector = Eigen::VectorXd;
  using ADvector = Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>;

  Objective(size_t vars, size_t consts, Polynomial<degree> poly)
    : vars_(vars)
    , consts_(consts)
    , poly_(std::move(poly))
    , derivative_(derive(poly_))
  {}
  
  ADvector::value_type cost(const ADvector& x) const {
    assert(x.size() == (int)vars_);
    return x[0] * x[0] - x[0];
  }

  ADvector constraints(const ADvector& ) const {
    ADvector v(consts_);
    return v;
  }

  Dvector init() const {
    return Dvector::Zero(vars_);
  }

  std::tuple<Dvector, Dvector> vbounds() const {
    Dvector lb = Dvector::Constant(vars_, -std::numeric_limits<double>::infinity());
    Dvector ub = Dvector::Constant(vars_, std::numeric_limits<double>::infinity());
    return std::make_tuple(std::move(lb), std::move(ub));
  }

  std::tuple<Dvector, Dvector> cbounds() const {
    Dvector lb = Dvector::Constant(consts_, -std::numeric_limits<double>::infinity());
    Dvector ub = Dvector::Constant(consts_, std::numeric_limits<double>::infinity());
    return std::make_tuple(std::move(lb), std::move(ub));
  }

  static std::string options() {
    static const std::string o = [] {
      std::string o;
      // turn off any printing
      o += "Integer print_level  0\n";
      o += "String  sb           yes\n";
  
      // maximum number of iterations
      o += "Integer max_iter     10\n";
      o += "Numeric tol          1e-6\n";

      // derivative testing
      o += "String  derivative_test none\n";
      // maximum amount of random pertubation; e.g., when evaluation finite diff
      o += "Numeric point_perturbation_radius  0.\n";

      // NOTE: Setting sparse to true allows the solver to take advantage
      // of sparse routines, this makes the computation MUCH FASTER. If you
      // can uncomment 1 of these and see if it makes a difference or not but
      // if you uncomment both the computation time should go up in orders of
      // magnitude.
      //o += "Sparse  true        forward\n";
      o += "Sparse  true        reverse\n";

      // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
      // Change this as you see fit.
      o += "Numeric max_cpu_time          0.25\n";
      return o;
    }();
    return o;
  }

private:
  const size_t vars_;
  const size_t consts_;
  const Polynomial<degree> poly_;
  const std::result_of<decltype(&derive<degree>)(const Polynomial<degree>&)>::type derivative_;
};
}

Control MPC::operator()(Model m) {
  auto poly = Polynomial<degree>(std::move(m.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(State::X) = Eigen::VectorXd::LinSpaced(10, -5, 50);
  waypoints.col(State::Y) = poly(waypoints.col(State::X));

  size_t Vars = 6;
  size_t Actuators = 2;
  
  auto solution = solve(Objective(N*Vars + (N - 1)*Actuators, N*Vars, std::move(poly)));
  if(solution.status != decltype(solution)::success) {
    std::cerr << "Failed to find solution for step " << m.count() << "\n";
  }
  std::cout << "Cost " << solution.obj_value << "\n";
    
  return Control(0, 0,
                 Eigen::MatrixXd(0, 2),
                 waypoints);
}

/*
vector<double> MPC::Solve(Eigen::VectorXd / *state* /, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 0;
  // TODO: Set the number of constraints
  size_t n_constraints = 0;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }



  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
*/
