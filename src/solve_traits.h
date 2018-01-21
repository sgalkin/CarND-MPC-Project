#pragma once

#include <limits>
#include <cppad/cppad.hpp>

template<template<typename S> class Vector, typename S>
struct solve_traits {
  using Scalar = S;
  using Dvector = Vector<Scalar>;
  using ADvector = Vector<CppAD::AD<Scalar>>;

  static constexpr Scalar unbounded = std::numeric_limits<Scalar>::infinity();
};
