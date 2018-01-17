#pragma once

#include <cstddef>
#include <utility>
#include <Eigen/Core>

class State {
public:
  State(double psim, double psiu,
        Eigen::MatrixXd wp,
        double speed, double angle, double throttle,
        Eigen::Vector2d p)
    : psim(psim), psiu(psiu)
    , wp(std::move(wp))
    , speed(speed), angle(angle), throttle(throttle)
    , p(p)
  {}
    
  
  State(const State& s) = default;
  State(State&& s) = default;
  
  State(State&& s, size_t count)
    : State(std::move(s)) {
    count_ = count;
  }

  State& operator= (State s) {
    std::swap(*this, s);
    return *this;
  }

  const double psim;
  const double psiu;
  const Eigen::MatrixXd wp;
  const double speed;
  const double angle;
  const double throttle;
  const Eigen::Vector2d p;
  
private:
  size_t count_{0};
};


class Count {
public:
  State operator()(State s) {
    return State(std::move(s), count_);
  }

private:
  size_t count_{0};
};
