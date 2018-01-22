#pragma once

#include <chrono>
#include "state.h"
#include "control.h"

class MPC {
public:
  using Interval = std::chrono::duration<double>;
  
  template<typename D>
  MPC(size_t N, D dt)
    : N(N), dt(std::chrono::duration_cast<Interval>(dt))
  {}
    
  Control operator()(State s);
  
private:
  size_t N;
  Interval dt;
};
