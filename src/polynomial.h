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
void ensureCompatibleVector(const T& x, const U& y) {
  static_assert(T::RowsAtCompileTime == U::RowsAtCompileTime,
                "x and y should have same number of rows");
  static_assert(T::ColsAtCompileTime == U::ColsAtCompileTime,
                "x and y should have same number of cols");
  static_assert(T::ColsAtCompileTime == 1 ||
                T::ColsAtCompileTime == Eigen::Dynamic,
                "x and y should be vectors");
  assert(x.rows() == y.rows());
  assert(x.cols() == y.cols());
  assert(x.cols() == 1);
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
class Polynomial {
  static_assert(P > 0, "P > 0 required");

public:
  using C = Eigen::Matrix<double, P + 1, 1>;

  explicit Polynomial(C c)
    : c_(std::move(c))
  {}

  template<typename T, typename U>
  Polynomial(const T& x, const U& y)
    : c_(details::fit<P, T, U>(x, y))
  {}

  template<typename T>
  Polynomial(const T& xy)
    : c_(details::fit<P>(xy.col(0), xy.col(1))) {
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
    return details::powers<P, T>(x) * c_;
  }

private:
  C c_;
};
