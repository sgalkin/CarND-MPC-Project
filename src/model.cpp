#include "model.h"

Model::Model(Eigen::MatrixXd wp, State state, Control actuator)
  : wp(std::move(wp))
  , state(std::move(state))
  , actuator(std::move(actuator))
{}
    
Model::Model(Model&& s, size_t count)
  : Model(std::move(s)) {
  count_ = count;
}

Model::Model(Model&& m, State state)
  : Model(std::move(m.wp), std::move(state), std::move(m.actuator))
{}

Model& Model::operator= (Model s) {
  std::swap(*this, s);
  return *this;
}

size_t Model::count() const { return count_; }
