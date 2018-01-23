#pragma once

#include <utility>
#include <type_traits>
#include <Eigen/Core>
#include <Eigen/QR>
#include "util.h"

namespace details {
template<size_t P, typename D,
         typename Result=typename Eigen::Matrix<typename D::Scalar, D::RowsAtCompileTime, P+1>>
  Result powers(const Eigen::MatrixBase<D>& x) {
  static_assert(D::ColsAtCompileTime == 1, "x is not a vector"); 
  Result R = Result::Constant(x.size(), P + 1, 1);
  for(size_t i = 1; i <= P; ++i)
    R.col(i) = R.col(i-1).array() * x.array();
  return R;
}

template<typename T, typename U>
bool ensureCompatibleVector(const T& x, const U& y) {
  static_assert(T::RowsAtCompileTime == U::RowsAtCompileTime,
                "x and y should have same number of rows");
  static_assert(T::ColsAtCompileTime == 1, "x should be vectors");
  static_assert(U::ColsAtCompileTime == 1, "y should be vectors");
  bool rows = x.rows() == y.rows();
  assert(rows);
  return rows;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
template<size_t P, typename T, typename U>
Eigen::Matrix<double, P+1, 1> fit(const T& x, const U& y) {
  static_assert(P >= 0, "P >= 0 required");
  ensureCompatibleVector(x, y);
  
  assert(P < (unsigned)x.size());
  auto Q = powers<P, T>(x).householderQr();
  return Q.solve(y);
}
}

template<size_t P>
struct Polynomial {
  static_assert(P >= 0, "P >= 0 required");

  using C = Eigen::Matrix<double, P + 1, 1>;

  explicit Polynomial(C c)
    : c(std::move(c))
  {}

  template<typename T, typename U>
  Polynomial(const T& x, const U& y)
    : c(details::fit<P, T, U>(x, y))
  {}

  template<typename T>
  explicit Polynomial(const T& xy)
    : c(details::fit<P>(xy.col(Axis::X), xy.col(Axis::Y))) {
    static_assert(T::ColsAtCompileTime == 2 ||
                  T::ColsAtCompileTime == Eigen::Dynamic,
                  "xy should have two columns");
    assert(xy.cols() == 2);
  }

  template<typename S,
           typename = typename std::enable_if<!std::is_base_of<Eigen::EigenBase<S>, S>::value>::type>
  S operator()(S x) const {
    S v = 0;
    S p = 1;
    for(size_t i = 0; i <= P; ++i) {
      v += c[i] * p;
      p *= x;
    }
    return v;
  }

  template<typename D>
  typename Eigen::Matrix<typename D::Scalar, D::RowsAtCompileTime, 1>
  operator()(const typename Eigen::MatrixBase<D>& x) const {
    static_assert(D::ColsAtCompileTime == 1, "x not a vector");
    return details::powers<P>(x) * c;
  }

  const C c;
};

template<size_t P>
Polynomial<P-1> derive(const Polynomial<P>& p) {
  typename Polynomial<P-1>::C c{
    Polynomial<P-1>::C::LinSpaced(P, 1, P).array() * p.c.block(1, 0, P, 1).array()
  };
  return Polynomial<P-1>(std::move(c));
}

inline Polynomial<0> derive(const Polynomial<0>&) {
  return Polynomial<0>((Polynomial<0>::C() << 0).finished());
}
