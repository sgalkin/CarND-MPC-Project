#pragma once

#include <chrono>
#include "model.h"
#include "control.h"

class MPC {
public:
  using Interval = std::chrono::duration<double>;
  
  MPC(size_t N, Interval dt)
    : N(N), dt(std::move(dt))
  {}
    
  Control operator()(Model s);
  
private:
  size_t N;
  Interval dt;
};
