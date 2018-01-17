#pragma once

#include <Eigen/Core>

struct Control {
  Control(double angle, double throttle,
          Eigen::MatrixXd prediction = Eigen::MatrixXd(0, 2),
          Eigen::MatrixXd wp = Eigen::MatrixXd(0, 2))
    : angle(angle), throttle(throttle)
    , prediction(std::move(prediction))
    , wp(std::move(wp))
  {}

  const double angle;
  const double throttle;
  const Eigen::MatrixXd prediction;
  const Eigen::MatrixXd wp;
};
