#pragma once

#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include "state.h"
#include "solve.h"
#include "solve_traits.h"
#include "polynomial.h"
#include "model_index.h"
#include "model_output.h"

namespace model {
namespace detail {
class Constraint {
  using Init = std::function<Solve::Dvector()>;
  using Polynomial = std::function<Solve::ADvector::value_type(Solve::ADvector::value_type)>;

public:
  using Dvector = Solve::Dvector;
  using ADvector = Solve::ADvector;

  template<typename I, typename P>
  explicit Constraint(I init, P p, size_t depth, double dt)
    : init_(std::move(init))
    , dp_([dp=derive(p)](ADvector::value_type x) { return dp(x); })
    , p_([p=std::move(p)](ADvector::value_type x) { return p(x); })
    , dt_(dt)
    , idx_(depth)
  {}
  
  ADvector operator()(const ADvector& x) const;  
  Dvector lower_bound() const;
  Dvector upper_bound() const;

private:
  const Init init_; 
  const Polynomial dp_;
  const Polynomial p_;
  const double dt_;
  const Index idx_;
};

class Variable {
  using Init = std::function<Solve::Dvector()>;
  using Bound = std::function<Solve::Dvector(Index, Solve::Dvector)>;

public:
  using Dvector = Solve::Dvector;

  template<typename I, typename LB, typename UB>
  explicit Variable(I init, size_t depth, LB lb, UB ub)
    : init_(std::move(init))
    , lb_([lb=std::move(lb)](Index idx, Dvector x) {
        return lb(idx, std::move(x));
      })
    , ub_([ub=std::move(ub)](Index idx, Dvector x) {
        return ub(idx, std::move(x));
      })
    , idx_(depth)
  {}

  Dvector operator()() const;
  Dvector lower_bound() const;
  Dvector upper_bound() const;
    
private:
  const Init init_;
  const Bound lb_;
  const Bound ub_;
  const Index idx_;
};

inline Solve::Dvector unchanged(Index, Solve::Dvector x) { return x; }
}

using Cost = std::function<Solve::ADvector::value_type(Index, const Solve::ADvector&)>;
using VariableBound = std::function<Solve::Dvector(Index, Solve::Dvector)>;
  
template<typename Polynomial>
State solve(State s, Polynomial p, size_t depth, double dt,
            Cost cost,
            VariableBound lower=detail::unchanged,
            VariableBound upper=detail::unchanged,
            const output::Output& out=output::No) {
  Index idx{depth};
  auto init = [s, p, dp=derive(p), idx]() {
    Solve::Dvector v{idx(LastState, 0)};
    v[idx(PSI, 0)] = s.psi;
    v[idx(V, 0)] = s.v;
    v[idx(X, 0)] = s.p(Axis::X);
    v[idx(Y, 0)] = s.p(Axis::Y);
    v[idx(CTE, 0)] = p(s.p(Axis::X)) - s.p(Axis::Y);
    v[idx(EPSI, 0)] = s.psi - atan(dp(s.p(Axis::X)));
    return v;
  };
  
  detail::Constraint constraint{init, std::move(p), depth, dt};
  detail::Variable variable{init, depth, std::move(lower), std::move(upper)};
    
  auto solution = ::solve(
    [idx, cost=std::move(cost)](const Solve::ADvector& x) {
      return cost(idx, x);
    }, constraint, variable);
  if(solution.status != decltype(solution)::success) {
    std::cerr << "Failed to find solution\n";
    return s;
  }
  out(solution.obj_value, idx, solution.x);

  Eigen::MatrixXd prediction(idx.depth, int(Axis::Plain));
  for(size_t i = 0; i < idx.depth; ++i) {
    prediction(i, Axis::X) = solution.x[idx(X, i)];
    prediction(i, Axis::Y) = solution.x[idx(Y, i)];
  }
  return State(solution.x[idx(PSI, 1)],
               solution.x[idx(V, 1)],
               (Eigen::Vector2d() << solution.x[idx(X, 1)],
                                     solution.x[idx(Y, 1)]).finished(),
               Control(solution.x[idx(S, 0)],
                       solution.x[idx(A, 0)],
                       std::move(prediction)));
}
}
