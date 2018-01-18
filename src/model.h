#pragma once

#include <cstddef>
#include <utility>
#include <Eigen/Core>
#include "control.h"

class State {
public:
  State(double psim, double psiu, double speed, Eigen::Vector2d p)
    : psim(psim), psiu(psiu), speed(speed), p(std::move(p))
  {}

  const double psim;
  const double psiu;
  const double speed;
  const Eigen::Vector2d p;  
};

class Model {
public:
  Model(Eigen::MatrixXd wp, State state, Control actuator)
    : wp(std::move(wp))
    , state(std::move(state))
    , actuator(std::move(actuator))
  {}
    
  Model(Model&& s, size_t count)
    : Model(std::move(s)) {
    count_ = count;
  }

  Model(Model&& m, State state)
    : Model(std::move(m.wp), std::move(state), std::move(m.actuator))
  {}

  Model(const Model& s) = default;
  Model(Model&& s) = default;
  
  Model& operator= (Model s) {
    std::swap(*this, s);
    return *this;
  }

  const Eigen::MatrixXd wp;
  const State state;
  const Control actuator;

  size_t count() const { return count_; }
  
private:
  size_t count_{0};
};

template<typename T>
class Count {
public:
  T operator()(T v) {
    return T(std::move(v), ++count_);
  }

private:
  size_t count_{0};
};
