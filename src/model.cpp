#include "model.h"

#include <cmath>
#include "drive.h"

namespace model {
namespace detail {
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
      
    auto ia = idx_(A, t-1);
    auto is = idx_(S, t-1);
      
    v[idx_(PSI, t)] -= x[ipsi] - d.psi(drive::D::v(x[iv]), drive::D::s(x[is]));
    v[idx_(V, t)] -= x[iv] + d.v(drive::D::a(x[ia]));
    v[idx_(X, t)] -= x[ix] + d.x(drive::D::psi(x[ipsi]), drive::D::v(x[iv]));
    v[idx_(Y, t)] -= x[iy] + d.y(drive::D::psi(x[ipsi]), drive::D::v(x[iv]));
    v[idx_(CTE, t)] -= p_(x[ix]) - x[iy] + d.y(drive::D::psi(x[iepsi]), drive::D::v(x[iv]));
    v[idx_(EPSI, t)] -= x[ipsi] - atan(dp_(x[ix])) - d.psi(drive::D::v(x[iv]), drive::D::s(x[is]));
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
