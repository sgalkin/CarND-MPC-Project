#pragma once

#include "state.h"
#include "solve.h"
#include "solve_traits.h"
#include "polynomial.h"

namespace model {
namespace detail {

template<typename S>
using Vector = Eigen::Matrix<S, Eigen::Dynamic, 1>;
using Solve = solve_traits<Vector, double>;

enum State { PSI = 0, V, X, Y, CTE, EPSI, LastState };
enum Actuator { A = 0, D, LastActuator };

struct Index {
  explicit Index(size_t depth);
    
  size_t operator()(State s, size_t i) const;
  size_t operator()(Actuator a, size_t i) const;

  size_t variables() const;
  size_t constraints() const;

  const size_t depth;
};

class Cost {
public:
  using ADvector = typename Solve::ADvector;
  
  explicit Cost(size_t depth);
  typename ADvector::value_type operator()(const ADvector& x) const;
  
private:    
  const Index idx;
};

class Constraint {
public:
  using Dvector = typename Solve::Dvector;
  using ADvector = typename Solve::ADvector;

  using Init = std::function<typename Dvector()>;
  using Polynomial = std::function<typename ADvector::value_type(ADvector::value_type)>;

  template<typename I, typename P>
  explicit Constraint(Init init, Polynomial p, size_t depth, double dt)
    : init_(init)
    , p_([poly](ADvector::value_type x) { return poly(x); })
    , dp_([p] {
        auto dp = poly.derive();
        return [dp](ADvector::value_type x) { return dp(x); };
      }())
    , ind_(depth)
  {}
  
  ADvector operator()(const ADvector& x) const;  
  Dvector lower_bound() const;
  Dvector upper_bound() const;

private:
  const Init init_;
  const Polynomial p;
  const Polynomial dp;
  const Index idx_;
};

class Variable {
  static constexpr double Dlow = -0.436332;//-25./180.*M_PI;
  static constexpr double Dhigh = -Dlow;

  static constexpr double Alow = -1;//-0.01;
  static constexpr double Ahigh = 1.;
    
public:
  using Dvector = typename Solve::Dvector;

  explicit Variable(Dvector x0, Index idx)
    : x0(std::move(x0))
    , idx(idx) {
    assert(size_t(this->x0.size()) == idx(LastState, 0));
  }

  Dvector operator()() const {
    Dvector v = Dvector::Zero(idx.variables());
    v.block(0, 0, idx(LastState, 0), 1) = x0;
    return v;
  }

  Dvector lower_bound() const {
    Dvector v = Dvector::Constant(idx.variables(), -Solve::unbounded);
    for(size_t i = 0; i < idx.depth - 1; ++i) {
      v[idx(A, i)] = Alow;
      v[idx(D, i)] = Dlow;
    }
    return v;
  }

  Dvector upper_bound() const {    
    Dvector v = -lower_bound();
    for(size_t i = 0; i < idx.depth - 1; ++i) {
      v[idx(A, i)] = Ahigh;
      v[idx(D, i)] = Dhigh;
    }
    return v;
  }
    
private:
  const Dvector x0;
  const Index idx;
};
}

template<typename Polynomial>
Solve::Dvector init(::State s, const Polynomial& p, size_t N) {
  Index idx{N};
  Solve::Dvector v{idx(model::LastState, 0)};
  v[idx(PSI, 0)] = s.psi;
  v[idx(V, 0)] = s.v;
  v[idx(X, 0)] = s.p(Axis::X);
  v[idx(Y, 0)] = s.p(Axis::Y);
  v[idx(CTE, 0)] = p(s.p(Axis::X)) - s.p(Axis::Y);
  v[idx(EPSI, 0)] = s.psi - atan(derive(p)(s.p(Axis::Y)));
  return v;
}

template<typename Polynomial>
State solve(State s, Polynomial p, size_t depth, double dt) {
  Index idx{depth};
  
  Cost cost{depth};
  Constraint constraint{depth, dt, 
//  const model::Solve::Dvector x0
    
    auto solution = ::solve(cost, constraint, variable);
  if(solution.status != decltype(solution)::success) {
      std::cerr << "Failed to find solution\n";
    }
    std::cout << "Cost: " << solution.obj_value << "\n";
    std::cout << solution.x.block(idx(model::LastState, 0), 0, model::LastState, 1) << "\n";
    std::cout << solution.x.block(idx(model::LastState, idx.depth - 1), 0, model::LastActuator, 1) << "\n";
    std::cout << solution.x[idx(model::D, 0)] << " " << solution.x[idx(model::A, 0)] << "\n";

    Eigen::MatrixXd prediction(idx.depth, int(Axis::Plain));
    for(size_t i = 0; i < idx.depth; ++i) {
      prediction(i, Axis::X) = solution.x[idx(model::X, i)];
      prediction(i, Axis::Y) = solution.x[idx(model::Y, i)];
    }
    return State(solution.x[idx(model::PSI, 1)],
                 solution.x[idx(model::V, 1)],
                 (Eigen::Vector2d() << solution.x[idx(model::X, 1)],
                                       solution.x[idx(model::Y, 1)]).finished(),
                 Control(solution.x[idx(model::D, 0)],
                         solution.x[idx(model::A, 0)],
                         std::move(prediction)));
  }

}
