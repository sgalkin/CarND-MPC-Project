#pragma once

#include "state.h"
#include "control.h"
#include <string>

struct Json {
  State operator()(std::string json);
  std::string operator()(Control control);
};
