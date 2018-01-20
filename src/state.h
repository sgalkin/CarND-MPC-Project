#pragma once

#include <Eigen/Core>

struct State {
  enum Index { X = 0, Y, V, PSI, Count};
    
  State(double psim/*, double psiu*/, double speed, Eigen::Vector2d p)
    : psim(psim)/*, psiu(psiu)*/, speed(speed), p(std::move(p))
  {}

  const double psim;
//  const double psiu;
  const double speed;
  const Eigen::Vector2d p;  
};
