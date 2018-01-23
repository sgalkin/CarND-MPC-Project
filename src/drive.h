#pragma once

namespace drive {
class D {
public:
  explicit D(double dt) : dt(dt) {}

  template<typename T>
  T x(T psi, T v) const { return v*cos(psi)*dt; }

  template<typename T>
  T y(T psi, T v) const { return v*sin(psi)*dt; }

  template<typename T>
  T v(T a) const { return a*dt; }

  template<typename T>
  T psi(T v, T d) const { return v/Lf*d*dt; }

private:
  const double dt;

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
};
}
