#include "mpc.h"

#include <Eigen/Core>

#include "model.h"
#include "control.h"
#include "polynomial.h"
#include "util.h"

namespace {
constexpr size_t degree{3};
}
namespace {
Eigen::Matrix2d rotation(double psi) {
  return (Eigen::Matrix2d() <<  cos(psi), -sin(psi),
                                sin(psi),  cos(psi)
    ).finished();
}

Eigen::MatrixXd rotate(Eigen::MatrixXd m, double psi, const Eigen::Vector2d& p) {
  return (m.rowwise() - p.transpose())*rotation(psi);
}
}

Control MPC::operator()(State s) {
  auto poly = Polynomial<degree>(std::move(s.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(Axis::X) = Eigen::VectorXd::LinSpaced(10, -5, 50);
  waypoints.col(Axis::Y) = poly(waypoints.col(Axis::X));

  auto cost = [](model::Index idx, const model::Solve::ADvector& x) {
    model::Solve::ADvector::value_type v = 0;
    for(size_t i = 0; i < idx.depth; ++i)
      v += pow(x[idx(model::V, i)] - 10, 2);
    return v;
  };

  auto bound = [](model::Solve::Dvector::value_type a,
                  model::Solve::Dvector::value_type d) {
    return [a, d](model::Index idx, model::Solve::Dvector v) {
      for(size_t i = 0; i < idx.depth - 1; ++i) {
        v[idx(model::A, i)] = a;
        v[idx(model::D, i)] = d;
      }
      return v;
    };
  };

  auto r = model::solve(std::move(s), std::move(poly), N, dt.count(),
                        bound(-0.01, -25*M_PI/180), bound(1, 25*M_PI/180),
                        cost);
  std::cerr << "wp\n" << waypoints << "\np\n" << r.current.prediction << "\n";
  return Control(-r.current.angle, r.current.throttle,
                 rotate(std::move(r.current.prediction), r.psi, r.p),
                 rotate(std::move(waypoints), r.psi, r.p));
}
