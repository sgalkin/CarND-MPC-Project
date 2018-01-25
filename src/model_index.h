#pragma once

#include <Eigen/Core>
#include "solve_traits.h"

namespace model {

template<typename S>
using Vector = Eigen::Matrix<S, Eigen::Dynamic, 1>;
using Solve = solve_traits<Vector, double>;

enum StateId { PSI = 0, V, X, Y, CTE, EPSI, LastState };
enum ActuatorId { A = 0, S, LastActuator };

struct Index {
  explicit Index(size_t depth);
    
  size_t operator()(StateId s, size_t i) const;
  size_t operator()(ActuatorId a, size_t i) const;

  size_t variables() const;
  size_t constraints() const;

  const size_t depth;
};

}
