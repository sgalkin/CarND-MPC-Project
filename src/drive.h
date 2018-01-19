#pragma once

#include <chrono>
#include "state.h"
#include "control.h"

State drive(State s, Control c, std::chrono::duration<double> dt);
