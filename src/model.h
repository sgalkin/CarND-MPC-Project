#pragma once

#include "state.h"
#include "drive.h"
#include "solve.h"
#include "solve_traits.h"
#include "polynomial.h"

namespace model {
template<typename S>
using Vector = Eigen::Matrix<S, Eigen::Dynamic, 1>;
using Solve = solve_traits<Vector, double>;

enum State { PSI = 0, V, X, Y, CTE, EPSI, LastState };
enum Actuator { A = 0, D, LastActuator };

struct Index {
  explicit Index(size_t depth) : depth(depth) {}
    
  size_t operator()(State s, size_t i) const {
    return assert(i < depth), i*int(LastState) + int(s);
  }
    
  size_t operator()(Actuator a, size_t i) const {
    return assert(i < depth - 1), depth*int(LastState) + i*int(LastActuator) + int(a);
  }

  size_t variables() const { return this->operator()(LastActuator, depth - 2); }
  size_t constraints() const { return this->operator()(LastState, depth - 1); }

  const size_t depth;
};

class Cost {
public:
  using ADvector = typename Solve::ADvector;
  
  explicit Cost(Index idx) : idx(idx) {}
  typename ADvector::value_type operator()(const ADvector& x) const {
    typename ADvector::value_type v = 0;
      
    for (size_t t = 0; t < idx.depth; ++t) {
      v += pow(x[idx(CTE, t)], 2);
      v += pow(x[idx(EPSI, t)], 2);
      // TODO: normalize
      v += pow(x[idx(V, t)] - 40, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < idx.depth - 1; ++t) {
      v += pow(x[idx(D, t)], 2);
      v += pow(x[idx(A, t)], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < idx.depth - 2; t++) {
      v += pow(x[idx(D, t + 1)] - x[idx(D, t)], 2);
      v += pow(x[idx(A, t + 1)] - x[idx(A, t)], 2);
    }

    return v;
  }
  
private:    
  const Index idx;
};

class Constraint {
public:
  using Dvector = typename Solve::Dvector;
  using ADvector = typename Solve::ADvector;
  using Polynomial = std::function<typename ADvector::value_type(ADvector::value_type)>;

  explicit Constraint(Dvector x0, Polynomial p, Polynomial dp, double dt, Index idx)
    : x0(std::move(x0))
    , p(std::move(p))
    , dp(std::move(dp))
    , dt(dt)
    , idx(idx) {
    assert(size_t(this->x0.size()) == idx(LastState, 0));
  }
  
  ADvector operator()(const ADvector& x) const {
    ADvector v = x.block(0, 0, idx.constraints(), 1);
    auto d = drive::D<typename ADvector::value_type>(dt);
    for(size_t t = 1; t < idx.depth; ++t) {
      auto ipsi = idx(PSI, t-1);
      auto iv = idx(V, t-1);
      auto ix = idx(X, t-1);
      auto iy = idx(Y, t-1);
//      auto icte = idx(CTE, t-1);
      auto iepsi = idx(EPSI, t-1);
      
      auto id = idx(D, t-1);
      auto ia = idx(A, t-1);
      
      v[idx(PSI, t)] -= x[ipsi] + d.psi(x[iv], x[id]);
      v[idx(V, t)] -= x[iv] + d.v(x[ia]);
      v[idx(X, t)] -= x[ix] + d.x(x[ipsi], x[iv]);
      v[idx(Y, t)] -= x[iy] + d.y(x[ipsi], x[iv]);
      v[idx(CTE, t)] -= p(x[ix]) - x[iy] + d.y(x[iepsi], x[iv]);
      v[idx(EPSI, t)] -= x[ipsi] - atan(dp(x[ix])) + d.psi(x[iv], x[id]);
    }
    return v;
  }
  
  Dvector lower_bound() const {
    Dvector v = Dvector::Zero(idx.constraints());
    v.block(0, 0, idx(LastState, 0), 1) = x0;
    return v;
  }
  
  Dvector upper_bound() const {
    return lower_bound();
  }

private:
  const Dvector x0;
  const Polynomial p;
  const Polynomial dp;
  const double dt;
  const Index idx;
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
    Dvector v = Dvector::Constant(idx.variables(), Solve::unbounded);
    for(size_t i = 0; i < idx.depth - 1; ++i) {
      v[idx(A, i)] = Alow;
      v[idx(D, i)] = Dlow;
    }
    return v;
  }

  Dvector upper_bound() const {
    Dvector v = Dvector::Constant(idx.variables(), Solve::unbounded);
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

template<size_t degree>
Solve::Dvector init(::State s, Polynomial<degree> p, size_t N) {
  Index idx{N};
  Solve::Dvector v{idx(model::LastState, 0)};
  v[idx(PSI, 0)] = s.psi;
  v[idx(V, 0)] = s.v;
  v[idx(X, 0)] = s.p(Axis::X);
  v[idx(Y, 0)] = s.p(Axis::Y);
  v[idx(CTE, 0)] = p(s.p(Axis::X)) - s.p(Axis::Y);
  v[idx(EPSI, 0)] = s.psi - atan(derive(p)(s.p(Axis::Y)));
  std::cerr << "init:v=" << v.size() << std::endl;
  return v;
}
}
template<size_t degree>
class Model {
public:
  Model(State s, Polynomial<degree> p, size_t N, double dt)
    : idx(N)
    , x0{model::init(s, p, N)}
    , cost{idx}
    , constraint{
        x0,
        [p](model::Solve::ADvector::value_type x) { return p(x); },
        [dp=derive(p)](model::Solve::ADvector::value_type x) { return dp(x); },
        dt, idx}
    , variable(x0, idx)
  {}

  Control solve() {
    std::cerr << "Model(" << idx.depth << "):\n"
              << "\tVariables: " << idx.variables() << "\n"
              << "\tConstraints: " << idx.constraints() << "\n";
    auto solution = ::solve(cost, constraint, variable);
    if(solution.status != decltype(solution)::success) {
      std::cerr << "Failed to find solution\n";
    }
    std::cout << "Cost " << solution.obj_value << "\n";

    Eigen::MatrixXd prediction(idx.depth, int(Axis::Plain));
    for(size_t i = 0; i < idx.depth; ++i) {
      prediction(i, Axis::X) = solution.x[idx(model::X, i)];
      prediction(i, Axis::Y) = solution.x[idx(model::Y, i)];
    }
    return Control(solution.x[idx(model::D, 0)],
                   solution.x[idx(model::A, 0)],
                   std::move(prediction));
  }
  
private:
  const model::Index idx;
  const model::Solve::Dvector x0;    
  const model::Cost cost;
  const model::Constraint constraint;
  const model::Variable variable;
};

template<size_t N, typename... Args>
Model<N> create_model(Args... args) { return Model<N>(std::forward<Args>(args)...); }

