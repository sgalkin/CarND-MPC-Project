#pragma once

#include <string>
#include "model.h"
#include "control.h"

struct Json {
  Model operator()(std::string json);
  std::string operator()(Control control);
};
