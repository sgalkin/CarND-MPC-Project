#pragma once

#include <utility>
#include <cppad/ipopt/solve.hpp>

template<typename Objective>
struct FG_eval {
  using ADvector = typename Objective::ADvector;

public:
  explicit FG_eval(Objective o) : o(std::move(o)) {}

  void operator()(ADvector& fg, const ADvector& x) const {
    fg[0] = o.cost(x);
    auto c = o.constraints(x);
    std::move(c.data(), c.data() + c.size(), std::next(fg.data()));
  }

private:
  const Objective o;
};

template<typename Objective>
CppAD::ipopt::solve_result<typename Objective::Dvector> solve(Objective o) {
  auto x = o.init();
  auto xb = o.vbounds();
  auto cb = o.cbounds();
  FG_eval<Objective> fg_eval(o);
    
  CppAD::ipopt::solve_result<typename Objective::Dvector> solution;
  CppAD::ipopt::solve(o.options(),
                      std::move(x), std::get<0>(xb), std::get<1>(xb),
                      std::get<0>(cb), std::get<1>(cb),
                      fg_eval,
                      solution);
  return solution;
}
