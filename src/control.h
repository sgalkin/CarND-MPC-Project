#pragma once

#include <Eigen/Core>
#include "util.h"

struct Control {
  static constexpr size_t dimensions = 2;
  
  Control(double angle, double throttle,
          Eigen::MatrixXd prediction = Eigen::MatrixXd(0, int(Axis::Plain)),
          Eigen::MatrixXd wp = Eigen::MatrixXd(0, int(Axis::Plain)))
    : angle(angle), throttle(throttle)
    , prediction(std::move(prediction))
    , wp(std::move(wp))
  {}

  const double angle;
  const double throttle;
  const Eigen::MatrixXd prediction;
  const Eigen::MatrixXd wp;
};
