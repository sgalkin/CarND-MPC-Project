#pragma once

#include "state.h"
#include "control.h"

struct Count {
  Control operator()(State /*s*/) { return Control(); }
};
