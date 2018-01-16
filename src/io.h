#pragma once

#include <string>

struct State;
struct Control;

struct Json {
  State operator()(std::string json);
  std::string operator()(Control control);
};
