#include "processor.h"

#include <Eigen/Core>
#include "drive.h"

namespace {
  static const size_t _coverage = 0;
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


Model Rotate::operator()(Model m) const {
  auto rwp = rotate(std::move(m.wp), m.state.psim, m.state.p);
  return Model(std::move(rwp), std::move(m.state), std::move(m.actuator));
}

Model Delay::operator()(Model m) const {
  auto s = drive(std::move(m.state), m.actuator, delay_); 
  return Model(std::move(m), std::move(s));
}
