#include "mpc.h"

#include <Eigen/Core>

#include "model.h"
#include "control.h"
#include "polynomial.h"
#include "util.h"

namespace {
constexpr size_t degree{3};
}

Control MPC::operator()(State s) {
  auto poly = Polynomial<degree>(std::move(s.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(Axis::X) = Eigen::VectorXd::LinSpaced(10, -5, 50);
  waypoints.col(Axis::Y) = poly(waypoints.col(Axis::X));

  auto c = create_model<degree>(std::move(s), std::move(poly), N, dt.count()).solve();
  return Control(c.angle, c.throttle, std::move(c.prediction), waypoints);
}
