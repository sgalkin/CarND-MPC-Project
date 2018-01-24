#pragma once

#include <utility>
#include <string>
#include <cppad/ipopt/solve.hpp>

struct DefaultOptions {
  std::string operator()() const {
    static const std::string o = [] {
      std::string o;
      // turn off any printing
      o += "Integer print_level                 0\n";
      o += "String  sb                          no\n";
  
      // maximum number of iterations
      //o += "Integer max_iter                    10\n";
      //o += "Numeric tol                         1e-6\n";

      // derivative testing
      o += "String  derivative_test             none\n";
      
      // maximum amount of random pertubation; e.g., when evaluation finite diff
      o += "Numeric point_perturbation_radius   0.\n";

      // NOTE: Setting sparse to true allows the solver to take advantage
      // of sparse routines, this makes the computation MUCH FASTER. If you
      // can uncomment 1 of these and see if it makes a difference or not but
      // if you uncomment both the computation time should go up in orders of
      // magnitude.
      o += "Sparse true        forward\n";
      //o += "Sparse  true                        reverse\n";

      // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
      // Change this as you see fit.
      // o += "Numeric max_cpu_time                0.25\n";
      return o;
    }();
    return o;
  }
};

template<typename Cost, typename Constraint>
class FG_eval {
public:
  using ADvector = typename Constraint::ADvector;

  FG_eval(Cost cost, Constraint constraint)
    : cost(std::move(cost))
    , constraint(std::move(constraint))
  {}

  void operator()(ADvector& fg, const ADvector& x) const {
    fg[0] = cost(x);
    auto c = constraint(x);
    std::move(c.data(), c.data() + c.size(), std::next(fg.data()));
  }

private:
  const Cost cost;
  const Constraint constraint;
};

template<typename Cost, typename Constraint>
FG_eval<Cost, Constraint> make_fg_eval(Cost cost, Constraint constraint) {
  return FG_eval<Cost, Constraint>(std::move(cost), std::move(constraint));
}

template<typename Cost, typename Constraint, typename Variable,
         typename Options=DefaultOptions,
         typename Result=CppAD::ipopt::solve_result<typename std::result_of<Variable()>::type>>
Result solve(Cost cost, Constraint constraint, Variable variable, Options options = {}) {
  auto fg = make_fg_eval(cost, constraint);
  Result solution;
  CppAD::ipopt::solve(options(),
                      variable(), variable.lower_bound(), variable.upper_bound(),
                      constraint.lower_bound(), constraint.upper_bound(),
                      fg,
                      solution);
  return solution;
}
