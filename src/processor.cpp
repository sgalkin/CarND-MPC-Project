#include "processor.h"

#include <Eigen/Core>
#include "drive.h"

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


State Rotate::operator()(State s) const {
  auto rwp = rotate(std::move(s.wp), s.psi, s.p);
  return State(0, s.v, Eigen::Vector2d::Zero(), std::move(s.current), std::move(rwp));
}


State Delay::operator()(State s) const {
  auto D = drive::D(delay_.count());
  return State{
    s.psi + D.psi(drive::D::v(s.v), drive::D::s(s.current.angle)),
    s.v + D.v(drive::D::a(s.current.throttle)),
    s.p + (Eigen::Vector2d() << D.x(drive::D::psi(s.psi), drive::D::v(s.v)),
                                D.y(drive::D::psi(s.psi), drive::D::v(s.v))).finished(),
    std::move(s.current),
    std::move(s.wp)
  };
  return s;
}
