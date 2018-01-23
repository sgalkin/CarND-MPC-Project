#include "model.h"
#include "drive.h"

namespace model {
namespace detail {

Index::Index(size_t depth) : depth(depth) {}

size_t Index::operator()(State s, size_t i) const {
  return assert(i < depth), i*int(LastState) + size_t(s);
}
    
size_t operator()(Actuator a, size_t i) const {
  return assert(i < depth - 1), depth*int(LastState) + i*int(LastActuator) + int(a);
}

  size_t variables() const { return this->operator()(LastActuator, depth - 2); }
  size_t constraints() const { return this->operator()(LastState, depth - 1); }

  const size_t depth;
};

  
// Cost function
typename Cost::ADvector::value_type Cost::operator()(const Cost::ADvector& x) const {
  typename Cost::ADvector::value_type v = 0;
      
  for (size_t t = 0; t < idx_.depth; ++t) {
    v += pow(x[idx_(CTE, t)], 2);
    v += pow(x[idx_(EPSI, t)], 2);
    // TODO: normalize
    v += pow(x[idx_(V, t)] - 40, 2);
  }

  // Minimize the use of actuators.
  for (size_t t = 0; t < idx_.depth - 1; ++t) {
    v += pow(x[idx_(D, t)], 2);
    v += pow(x[idx_(A, t)], 2);
  }

  // Minimize the value gap between sequential actuations.
  for (size_t t = 0; t < idx_.depth - 2; t++) {
    v += pow(x[idx_(D, t + 1)] - x[idx_(D, t)], 2);
    v += pow(x[idx_(A, t + 1)] - x[idx_(A, t)], 2);
  }
  return v;
}


// Constraints
Contstaint::ADvector operator()(const Constraint::ADvector& x) const {
  ADvector v = x.block(0, 0, idx.constraints(), 1);
  auto d = drive::D(dt);

  for(size_t t = 1; t < idx.depth; ++t) {
    auto ipsi = idx(PSI, t-1);
    auto iv = idx(V, t-1);
    auto ix = idx(X, t-1);
    auto iy = idx(Y, t-1);
    auto iepsi = idx(EPSI, t-1);
      
    auto id = idx(D, t-1);
    auto ia = idx(A, t-1);
      
    v[idx(PSI, t)] -= x[ipsi] + d.psi(x[iv], x[id]);
    v[idx(V, t)] -= x[iv] + d.v(x[ia]);
    v[idx(X, t)] -= x[ix] + d.x(x[ipsi], x[iv]);
    v[idx(Y, t)] -= x[iy] + d.y(x[ipsi], x[iv]);
    v[idx(CTE, t)] -= p_(x[ix]) - x[iy] + d.y(x[iepsi], x[iv]);
    v[idx(EPSI, t)] -= x[ipsi] - atan(dp_(x[ix])) + d.psi(x[iv], x[id]);
  }
  return v;
}
  
Contstaint::Dvector Contstaint::lower_bound() const {
  Dvector v = Dvector::Zero(idx.constraints());
  v.block(0, 0, idx(LastState, 0), 1) = init_();
  return v;
}
  
Contstaint::Dvector Contstaint::upper_bound() const {
  return lower_bound();
}

}
}
