#pragma once

#include <cstddef>
#include <utility>
#include <chrono>
#include "state.h"

template<typename T>
class Count {
public:
  T operator()(T v) {
    return T(std::move(v), ++count_);
  }

private:
  size_t count_{0};
};

struct Rotate {
  State operator()(State m) const;
};

using Interval = std::chrono::duration<double>;
class Delay {
public:
  template<typename D>
  explicit Delay(D delay) :
    delay_(std::chrono::duration_cast<Interval>(delay))
  {}
  
  State operator()(State m) const;
  
private:
  Interval delay_;
};
