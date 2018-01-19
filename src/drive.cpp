#include "drive.h"

namespace {
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
constexpr double Lf = 2.67;
}

State drive(State s, Control c, std::chrono::duration<double> dt) {
  return State{
    s.psim + s.speed/Lf*c.angle*dt.count(),
    s.psiu + s.speed/Lf*c.angle*dt.count(),
    s.speed + c.throttle*dt.count(),
    s.p + s.speed*dt.count()*(Eigen::Vector2d() << cos(s.psim), sin(s.psim)).finished()
  };
}
