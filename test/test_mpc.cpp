#include "catch.hpp"
#include "mpc.h"
#include "solve.h"
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cmath>
#include <limits>
#include <iostream>
#if 0
class Objective {
public:
  using Dvector = Eigen::VectorXd;
  using ADvector = Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>;

  Objective(size_t variables, size_t constraints)
    : variables_(variables)
    , constraints_(constraints)
  {}
  
  ADvector::value_type cost(const ADvector& x) const {
    assert(x.size() == (int)variables_);
    return x[0] * x[0] - x[0];
  }

  ADvector constraints(const ADvector& ) const {
    ADvector v(constraints_);
    return v;
  }

  Dvector init() const {
    return Dvector::Zero(variables_);
  }

  std::tuple<Dvector, Dvector> vbounds() const {
    Dvector lb = Dvector::Constant(variables_, -std::numeric_limits<double>::infinity());
    Dvector ub = Dvector::Constant(variables_, std::numeric_limits<double>::infinity());
    return std::make_tuple(std::move(lb), std::move(ub));
  }

  std::tuple<Dvector, Dvector> cbounds() const {
    Dvector lb = Dvector::Constant(constraints_, -std::numeric_limits<double>::infinity());
    Dvector ub = Dvector::Constant(constraints_, std::numeric_limits<double>::infinity());
    return std::make_tuple(std::move(lb), std::move(ub));
  }

  static std::string options() {
    static const std::string o = [] {
      std::string o;
      // turn off any printing
      //o += "Integer print_level  4\n";
      //o += "String  sb           yes\n";
  
      // maximum number of iterations
      o += "Integer max_iter     10\n";
      o += "Numeric tol          1e-6\n";

      // derivative testing
      o += "String  derivative_test none\n";//           second-order\n";
      // maximum amount of random pertubation; e.g., when evaluation finite diff
      o += "Numeric point_perturbation_radius  0.\n";

      // NOTE: Setting sparse to true allows the solver to take advantage
      // of sparse routines, this makes the computation MUCH FASTER. If you
      // can uncomment 1 of these and see if it makes a difference or not but
      // if you uncomment both the computation time should go up in orders of
      // magnitude.
      o += "Sparse  true        forward\n";
      o += "Sparse  true        reverse\n";

      // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
      // Change this as you see fit.
      o += "Numeric max_cpu_time          0.5\n";
      return o;
    }();
    return o;
  }
  
private:
  const size_t variables_;
  const size_t constraints_;
};


TEST_CASE("Solve") {
  auto solution = solve(Objective(1, 0));
  //
  // Check some of the solution values
  //
  bool ok = solution.status == decltype(solution)::success;
  std::cerr << solution.x << std::endl;
//  REQUIRE(false);
}
#endif
