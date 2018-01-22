#include "mpc.h"

#include <iostream>
#include <Eigen/Core>

//#include "model.h"
#include "control.h"
#include "polynomial.h"
#include "util.h"

#include "drive.h"
#include "solve.h"
#include "solve_traits.h"

namespace {
constexpr size_t degree{3};

template<size_t degree>
class Model {
  struct Index {
    enum State { PSI = 0, V, X, Y, LastState };
    enum Actuator { A = 0, D, LastActuator };

    explicit Index(size_t N) : N(N) {}
    
    size_t operator()(State s, size_t i) const {
      return assert(i < N), i*int(LastState) + int(s);
    }
    
    size_t operator()(Actuator a, size_t i) const {
      return assert(i < N - 1), N*int(LastState) + i*int(LastActuator) + int(a);
    }

    size_t size() const { return N; }
    size_t variables() const { return this->operator()(LastActuator, N - 2); }
    size_t constraints() const { return this->operator()(LastState, N - 1); }
    
  private:
    const size_t N;
  };
  
public:
  Model(State s, Polynomial<degree>/* p*/, size_t N, MPC::Interval dt)
    : idx(N)
    , cost(idx)
    , constraint(s, dt.count(), idx)
    , variable(s, idx)
  {}

  Control solve() {
    std::cerr << "Model(" << idx.size() << "):\n"
              << "\tVariables: " << idx.variables() << "\n"
              << "\tConstraints: " << idx.constraints() << "\n";
    auto solution = ::solve(cost, constraint, variable);
    if(solution.status != decltype(solution)::success) {
      std::cerr << "Failed to find solution\n";
    }
    std::cout << "Cost " << solution.obj_value << "\n";

    Eigen::MatrixXd prediction(idx.size(), int(Axis::Plain));
    for(size_t i = 0; i < idx.size(); ++i) {
      prediction(i, Axis::X) = solution.x[idx(Index::X, i)];
      prediction(i, Axis::Y) = solution.x[idx(Index::Y, i)];
    }
    return Control(solution.x[idx(Index::D, 0)],
                   solution.x[idx(Index::A, 0)],
                   std::move(prediction));
  }
  
private:
  const Index idx;
  
private:
  template<typename S>
  using Vector = Eigen::Matrix<S, Eigen::Dynamic, 1>;
  using Solve = solve_traits<Vector, double>;

  const struct Cost {
    using ADvector = typename Solve::ADvector;
    explicit Cost(Index idx) : idx(idx) {}
    typename ADvector::value_type operator()(const ADvector& x) const {
      typename ADvector::value_type v = 0;
      
      for (size_t t = 0; t < idx.size(); t++) {
//        v += CppAD::pow(x[idx(cte_start + t], 2);
//                       fg[0] += CppAD::pow(vars[epsi_start + t], 2);
        // TODO: normalize
        v += CppAD::pow(x[idx(Index::V, t)] - 50, 2);
      }

      // Minimize the use of actuators.
      for (size_t t = 0; t < idx.size() - 1; t++) {
        v += CppAD::pow(x[idx(Index::D, t)], 2);
        v += CppAD::pow(x[idx(Index::A, t)], 2);
      }

      // Minimize the value gap between sequential actuations.
      for (size_t t = 0; t < idx.size() - 2; t++) {
        v += CppAD::pow(x[idx(Index::D, t + 1)] - x[idx(Index::D, t)], 2);
        v += CppAD::pow(x[idx(Index::A, t + 1)] - x[idx(Index::A, t)], 2);
      }

      return v;
    }
    
    const Index idx;
  } cost;

  const class Constraint {
  public:
    using Dvector = typename Solve::Dvector;
    using ADvector = typename Solve::ADvector;

    explicit Constraint(State s, double dt, Index idx)
      : s(std::move(s)), dt(dt), idx(idx)
    {}
    ADvector operator()(const ADvector& x) const {
      ADvector v = x.block(0, 0, idx.constraints(), 1);
      std::cerr << "Const: " << v.rows() << " x " << v.cols() << "\n";
      auto D = drive::D<typename ADvector::value_type>(dt);
      for(size_t t = 1; t < idx.size(); ++t) {
        std::cerr << t << std::endl;
        auto ipsi = idx(Index::PSI, t-1);
        auto iv = idx(Index::V, t-1);
        auto ix = idx(Index::X, t-1);
        auto iy = idx(Index::Y, t-1);
        auto id = idx(Index::D, t-1);
        auto ia = idx(Index::A, t-1);
        v[idx(Index::PSI, t)] -= v[ipsi] + D.psi(v[iv], x[id]);
        v[idx(Index::V, t)] -= v[iv] + D.v(x[ia]);
        v[idx(Index::X, t)] -= v[ix] + D.x(v[ipsi], v[iv]);
        v[idx(Index::Y, t)] -= v[iy] + D.y(v[ipsi], v[iv]);
      }
      return v;
    }
    Dvector lower_bound() const {
      Dvector v = Dvector::Zero(idx.constraints());
      v[idx(Index::PSI, 0)] = s.psi;
      v[idx(Index::V, 0)] = s.v;
      v[idx(Index::X, 0)] = s.p(Axis::X);
      v[idx(Index::Y, 0)] = s.p(Axis::Y);
      return v;
    }
    Dvector upper_bound() const {
      return lower_bound();
    }

  private:
    const State s;
    const double dt;
    const Index idx;
  } constraint;

  const class Variable {
    static constexpr double Dlow = -25./180.*M_PI;
    static constexpr double Dhigh = -Dlow;

    static constexpr double Alow = -0.01;
    static constexpr double Ahigh = 1.;
    
  public:
    using Dvector = typename Solve::Dvector;

    explicit Variable(State s, Index idx) : s(std::move(s)), idx(idx) {}
    Dvector operator()() const {
      Dvector v = Dvector::Zero(idx.variables());
      v[idx(Index::PSI, 0)] = s.psi;
      v[idx(Index::V, 0)] = s.v;
      v[idx(Index::X, 0)] = s.p(Axis::X);
      v[idx(Index::Y, 0)] = s.p(Axis::Y);
      return v;
    }
    Dvector lower_bound() const {
      Dvector v = Dvector::Constant(idx.variables(), Solve::unbounded);
      for(size_t i = 0; i < idx.size() - 1; ++i) {
        v[idx(Index::A, i)] = Alow;
        v[idx(Index::D, i)] = Dlow;
      }
      return v;
    }
    Dvector upper_bound() const {
      Dvector v = Dvector::Constant(idx.variables(), Solve::unbounded);
      for(size_t i = 0; i < idx.size() - 1; ++i) {
        v[idx(Index::A, i)] = Ahigh;
        v[idx(Index::D, i)] = Dhigh;
      }
      return v;
    }
    
  private:
    const State s;
    const Index idx;
  } variable;
};

template<size_t N, typename... Args>
Model<N> create_model(Args... args) { return Model<N>(std::forward<Args>(args)...); }
}

Control MPC::operator()(State s) {
  auto poly = Polynomial<degree>(std::move(s.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(Axis::X) = Eigen::VectorXd::LinSpaced(10, -5, 50);
  waypoints.col(Axis::Y) = poly(waypoints.col(Axis::X));

  auto c = create_model<degree>(std::move(s), std::move(poly), N, dt).solve();
  return Control(c.angle, c.throttle, std::move(c.prediction), waypoints);
}
