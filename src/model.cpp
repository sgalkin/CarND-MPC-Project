#include "model.h"

#include <cmath>
#include "drive.h"

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

namespace detail {
// Cost function
typename Cost::ADvector::value_type Cost::operator()(const Cost::ADvector& x) const {
  typename Cost::ADvector::value_type v = 0;
      
  for (size_t t = 0; t < idx_.depth; ++t) {
    v += pow(x[idx_(CTE, t)], 2);
    v += pow(x[idx_(EPSI, t)], 2);
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
  return v + extra_(idx_, x);
}


// Constraints
Constraint::ADvector Constraint::operator()(const Constraint::ADvector& x) const {
  ADvector v = x.block(0, 0, idx_.constraints(), 1);
  auto d = drive::D(dt_);

  for(size_t t = 1; t < idx_.depth; ++t) {
    auto ipsi = idx_(PSI, t-1);
    auto iv = idx_(V, t-1);
    auto ix = idx_(X, t-1);
    auto iy = idx_(Y, t-1);
    auto iepsi = idx_(EPSI, t-1);
      
    auto id = idx_(D, t-1);
    auto ia = idx_(A, t-1);
      
    v[idx_(PSI, t)] -= x[ipsi] + d.psi(x[iv], x[id]);
    v[idx_(V, t)] -= x[iv] + d.v(x[ia]);
    v[idx_(X, t)] -= x[ix] + d.x(x[ipsi], x[iv]);
    v[idx_(Y, t)] -= x[iy] + d.y(x[ipsi], x[iv]);
    v[idx_(CTE, t)] -= p_(x[ix]) - x[iy] + d.y(x[iepsi], x[iv]);
    v[idx_(EPSI, t)] -= x[ipsi] - atan(dp_(x[ix])) + d.psi(x[iv], x[id]);
  }
  return v;
}
  
Constraint::Dvector Constraint::lower_bound() const {
  Dvector v = Dvector::Zero(idx_.constraints());
  v.block(0, 0, idx_(LastState, 0), 1) = init_();
  return v;
}
  
Constraint::Dvector Constraint::upper_bound() const {
  return lower_bound();
}


// Variable
Variable::Dvector Variable::operator()() const {
  Dvector v = Dvector::Zero(idx_.variables());
  v.block(0, 0, idx_(LastState, 0), 1) = init_();
  return v;
}

Variable::Dvector Variable::lower_bound() const {
  Dvector v = Dvector::Constant(idx_.variables(), -Solve::unbounded);
  return lb_(idx_, std::move(v));
}

Variable::Dvector Variable::upper_bound() const {
  Dvector v = -lower_bound();
  return ub_(idx_, std::move(v));
}

}
}
