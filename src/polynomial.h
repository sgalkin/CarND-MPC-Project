#pragma once

#include <utility>
#include <Eigen/Core>
#include <Eigen/QR>

namespace details {
template<size_t P, typename T, int M=T::RowsAtCompileTime==Eigen::Dynamic ? Eigen::Dynamic : P+1>
Eigen::Matrix<double, T::RowsAtCompileTime, M> powers(const T& x) {
  static_assert(M == Eigen::Dynamic || M == P + 1, "Invalid number of columns");
  
  using Result = Eigen::Matrix<double, T::RowsAtCompileTime, M>;
  Result R = Result::Constant(x.size(), P + 1, 1);
  for(size_t i = 1; i <= P; ++i)
    R.col(i) = R.col(i-1).array() * x.array();
  return R;
}

template<typename T, typename U>
bool ensureCompatibleVector(const T& x, const U& y) {
  static_assert(T::RowsAtCompileTime == U::RowsAtCompileTime,
                "x and y should have same number of rows");
  static_assert(T::ColsAtCompileTime == U::ColsAtCompileTime,
                "x and y should have same number of cols");
  static_assert(T::ColsAtCompileTime == 1 ||
                T::ColsAtCompileTime == Eigen::Dynamic,
                "x and y should be vectors");
  bool rows = x.rows() == y.rows();
  bool cols = x.cols() == y.cols();
  bool vector = x.cols() == 1;
  assert(rows);
  assert(cols);
  assert(vector);
  return rows && cols && vector;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
template<size_t P, typename T, typename U>
Eigen::Matrix<double, P+1, 1> fit(const T& x, const U& y) {
  static_assert(P > 0, "P > 0 required");
  ensureCompatibleVector(x, y);
  
  assert(P < (unsigned)x.size());
  auto Q = powers<P, T>(x).householderQr();
  return Q.solve(y);
}
}


template<size_t P>
struct Polynomial {
  static_assert(P >= 0, "P > 0 required");

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
    : c(details::fit<P>(xy.col(0), xy.col(1))) {
    static_assert(T::ColsAtCompileTime == 2 ||
                  T::ColsAtCompileTime == Eigen::Dynamic,
                  "xy should have two columns");
    assert(xy.cols() == 2);
  }

  double operator()(double x) const {
    return this->operator()((Eigen::Matrix<double, 1, 1>() << x).finished())(0);
  }

  template<typename T>
  Eigen::Matrix<double, T::RowsAtCompileTime, 1> operator()(const T& x) const {
    return details::powers<P, T>(x) * c;
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

Polynomial<0> derive(const Polynomial<0>&) {
  return Polynomial<0>((Polynomial<0>::C() << 0).finished());
}
