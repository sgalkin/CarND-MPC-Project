#pragma once

#include <string>
#include "state.h"
#include "control.h"

struct Json {
  State operator()(std::string json);
  std::string operator()(Control control);
};
