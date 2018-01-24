#include "mpc.h"

#include <Eigen/Core>

#include "model.h"
#include "control.h"
#include "polynomial.h"
#include "util.h"

namespace {
constexpr size_t degree{3};

constexpr double Alb = -0.01;
constexpr double Aub = 1;

constexpr double Slb = -25.*M_PI/180.;
constexpr double Sub = -Slb;  

model::Solve::ADvector::value_type cost(model::Index idx, const model::Solve::ADvector& x) {
  model::Solve::ADvector::value_type v = 0;
  for (size_t t = 0; t < idx.depth; ++t) {
    v += pow(x[idx(model::CTE, t)], 2);
    v += pow(x[idx(model::EPSI, t)], 2);
    v += pow(x[idx(model::V, t)] - 50, 2);
//    v += 10/(1e-2 + abs(x[idx(model::V, t)]));
  }
  // Minimize the use of actuators.
  for (size_t t = 0; t < idx.depth - 1; ++t) {
    v += 100*pow(x[idx(model::A, t)], 2);
    v += pow(x[idx(model::S, t)], 2);
  }
  // Minimize the value gap between sequential actuations.
  for (size_t t = 0; t < idx.depth - 2; t++) {
    v += pow(x[idx(model::A, t + 1)] - x[idx(model::A, t)], 2);
    v += 100*pow(x[idx(model::S, t + 1)] - x[idx(model::S, t)], 2);
  }
  return v;  
}
}


Control MPC::operator()(State s) {
  auto poly = Polynomial<degree>(std::move(s.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(Axis::X) = Eigen::VectorXd::LinSpaced(10, 5, 80);
  waypoints.col(Axis::Y) = poly(waypoints.col(Axis::X));

  auto bind = [](model::Solve::Dvector::value_type a,
                 model::Solve::Dvector::value_type s) {
    return [a, s](model::Index idx, model::Solve::Dvector v) {
      for(size_t i = 0; i < idx.depth - 1; ++i) {
        v[idx(model::A, i)] = a;
        v[idx(model::S, i)] = s;
      }
      return v;
    };
  };

  auto r = model::solve(std::move(s), std::move(poly), N, dt.count(),
                        cost, bind(Alb, Slb), bind(Aub, Sub), model::stdOutput);

  return Control(r.current.angle, r.current.throttle,
                 std::move(r.current.prediction),
                 std::move(waypoints));
}
