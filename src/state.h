#pragma once

#include <Eigen/Core>
#include "control.h"

struct State {
  static constexpr size_t dimensions = 4;
  
  State(double psi/*, double psiu*/, double v, Eigen::Vector2d p, Control c, Eigen::MatrixXd wp)
    : psi(psi)/*, psiu(psiu)*/, v(v), p(std::move(p))
    , current(std::move(c)), wp(std::move(wp))
  {}

  const double psi;
//  const double psiu;
  const double v;
  const Eigen::Vector2d p;
  const Control current;
  
  const Eigen::MatrixXd wp;
};
