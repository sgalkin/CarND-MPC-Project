#pragma once

#include <cstddef>
#include <utility>

class State {
public:
  State() {}
  
  State(const State& s) = default;
  State(State&& s) = default;
  
  State(State&& s, size_t count)
    : State(std::move(s)) {
    count_ = count;
  }

  State& operator= (State s) {
    std::swap(*this, s);
    return *this;
  }
  
private:
  size_t count_{0};
};


class Count {
public:
  State operator()(State s) {
    return State(std::move(s), count_);
  }

private:
  size_t count_{0};
};
