#pragma once

#include <cstddef>
#include <Eigen/Core>
#include "state.h"
#include "control.h"

class Model {
public:
  Model(Eigen::MatrixXd wp, State state, Control actuator);
    
  Model(Model&& s, size_t count);
  Model(Model&& m, State state);

  Model(const Model& s) = default;
  Model(Model&& s) = default;
  
  Model& operator= (Model s);

  const Eigen::MatrixXd wp;
  const State state;
  const Control actuator;

  size_t count() const;
  
private:
  size_t count_{0};
};
