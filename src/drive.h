#pragma once

namespace drive {
namespace detail {
template<typename T, int X>
struct NamedArgument {
public:
  using Base = T;
  explicit NamedArgument(T v) : v(std::move(v)) {}  
  const T& value() const { return v; }
private:
  T v;
};
}

class D {
public:
  template<typename T> using PSI = detail::NamedArgument<T, 0>;
  template<typename T> using V = detail::NamedArgument<T, 1>;
  template<typename T> using A = detail::NamedArgument<T, 2>;
  template<typename T> using S = detail::NamedArgument<T, 3>;

  template<typename T> static PSI<T> psi(T v) { return PSI<T>(v); }
  template<typename T> static V<T> v(T v) { return V<T>(v); }
  template<typename T> static A<T> a(T v) { return A<T>(v); }
  template<typename T> static S<T> s(T v) { return S<T>(v); }

  explicit D(double dt) : dt(dt) {}

  template<typename T>
  T x(PSI<T> psi, V<T> v) const { return v.value()*cos(psi.value())*dt; }

  template<typename T>
  T y(PSI<T> psi, V<T> v) const { return v.value()*sin(psi.value())*dt; }

  template<typename T>
  T v(A<T> a) const { return a.value()*dt; }

  template<typename T>
  T psi(V<T> v, S<T> s) const { return v.value()/Lf*s.value()*dt; }

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
  const double Lf = 2.67;
};
}
