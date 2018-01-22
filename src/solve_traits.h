#pragma once

#include <limits>
#include <cppad/cppad.hpp>

template<template<typename S> class Vector, typename S>
struct solve_traits {
  using Scalar = S;
  using Dvector = Vector<Scalar>;
  using ADvector = Vector<CppAD::AD<Scalar>>;

  static const Scalar unbounded;
};

template<template<typename S> class Vector, typename S>
const typename solve_traits<Vector, S>::Scalar solve_traits<Vector, S>::unbounded = 
  std::numeric_limits<typename solve_traits<Vector, S>::Scalar>::infinity();

