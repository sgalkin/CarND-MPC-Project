#pragma once

namespace drive {
template<typename T>
struct D {
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
  static constexpr double Lf = 2.67;

  explicit D(double dt) : dt(dt) {}

  T x(T psi, T v) const { return v*cos(psi)*dt; }
  T y(T psi, T v) const { return v*sin(psi)*dt; }
  T v(T a) const { return a*dt; }
  T psi(T v, T d) const { return v/Lf*d*dt; }

  const double dt;
};
}
