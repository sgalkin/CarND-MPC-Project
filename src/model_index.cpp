#include "model_index.h"

namespace model {

Index::Index(size_t depth) : depth(depth) {}
size_t Index::operator()(StateId s, size_t i) const {
  return assert(i < depth), i*int(LastState) + size_t(s);
}
size_t Index::operator()(ActuatorId a, size_t i) const {
  return assert(i < depth - 1), depth*int(LastState) + i*int(LastActuator) + int(a);
}

size_t Index::variables() const {
  return this->operator()(LastActuator, depth - 2);
}
size_t Index::constraints() const {
  return this->operator()(LastState, depth - 1);
}

}
