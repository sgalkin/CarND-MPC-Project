#pragma once

#include <iostream>
#include <chrono>
#include "state.h"
#include "control.h"
#include "model_output.h"
#include "stream_holder.h"

class MPC {
public:
  using Interval = std::chrono::duration<double>;
  
  template<typename D>
  MPC(size_t N, D dt, StreamHolder& stream)
    : N(N), dt(std::chrono::duration_cast<Interval>(dt))
    , output{&(*stream) == &std::cout ?
             model::output::Output{model::output::brief(*stream)} :
             model::output::Output{model::output::csv(*stream)}}
  {}
    
  Control operator()(State s);
  
private:
  size_t N;
  Interval dt;
  model::output::Output output;
};
