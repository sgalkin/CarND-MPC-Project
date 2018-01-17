#pragma once

#include <iostream>
#include <utility>
#include <Eigen/Core>
#include <Eigen/QR>

namespace details {
template<size_t P, int N=Eigen::Dynamic, int M=(N==Eigen::Dynamic? N : P + 1)>
Eigen::Matrix<double, N, M> powers(const Eigen::Matrix<double, N, 1>& x) {
  static_assert(M == Eigen::Dynamic ||
                M == P + 1, "Invalid number of columns");
  Eigen::Matrix<double, N, M> R = Eigen::Matrix<double, N, M>::Constant(x.size(), P + 1, 1);
  for(size_t i = 1; i <= P; ++i)
    R.col(i) = R.col(i-1).array() * x.array();
  return R;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
template<size_t P, int N=Eigen::Dynamic>
Eigen::Matrix<double, P+1, 1> fit(const Eigen::Matrix<double, N, 1>& x,
                                  const Eigen::Matrix<double, N, 1>& y) {
  static_assert(P > 0, "P > 0 required");
  assert(x.size() == y.size());
  assert(P < (unsigned)x.size());
  auto Q = powers<P, N>(x).householderQr();
  return Q.solve(y);
}
}

template<size_t P>
class Polynomial {
  static_assert(P > 0, "P > 0 required");

public:
  using C = Eigen::Matrix<double, P + 1, 1>;

  explicit Polynomial(C c)
    : c_(std::move(c)) {}

  template<int N=Eigen::Dynamic>
  Polynomial(const Eigen::Matrix<double, N, 1>& x,
             const Eigen::Matrix<double, N, 1>& y)
    : c_(details::fit<P>(x, y))
  {}

  double operator()(double x) const {
    return this->operator()<1>(
      (Eigen::Matrix<double, 1, 1>() << x).finished())(0);
  }

  template<int N=Eigen::Dynamic>
  Eigen::Matrix<double, N, 1> operator()(
    const Eigen::Matrix<double, N, 1>& x) const {
    return details::powers<P, N>(x) * c_;
  }

private:
  C c_;
};
