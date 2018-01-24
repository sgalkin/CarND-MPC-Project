#include "catch.hpp"

#include <iostream>
#include "model.h"
#include "util.h"

TEST_CASE("Model") {
  constexpr size_t degree = 1;
  constexpr size_t N = 25;
  constexpr double dt = 0.05;

  constexpr double Vref = 40;
  
  constexpr double Dlow = -25.*M_PI/180.;
  constexpr double Dhigh = -Dlow;

  constexpr double Alow = -1.;
  constexpr double Ahigh = 1.;


  auto cost = [](model::Index idx, const model::Solve::ADvector& x) {
    model::Solve::ADvector::value_type v = 0;
    for(size_t i = 0; i < idx.depth; ++i)
      v += pow(x[idx(model::V, i)] - Vref, 2);
    return v;
  };

  auto bound = [](model::Solve::Dvector::value_type a,
                  model::Solve::Dvector::value_type d) {
    return [a, d](model::Index idx, model::Solve::Dvector v) {
      for(size_t i = 0; i < idx.depth - 1; ++i) {
        v[idx(model::A, i)] = a;
        v[idx(model::D, i)] = d;
      }
      return v;
    };
  };

  auto pts = (Eigen::MatrixXd(2, int(Axis::Plain)) << -100, -1,
                                                       100, -1).finished();

  auto c = Polynomial<degree>(std::move(pts));

  constexpr double psi = 0;
  constexpr double v = 10;
  const auto xy = (Eigen::Vector2d() << -1, 10).finished();

  SECTION("01 iteration") {
    State init(psi, v, xy, Control(0, 0));
    auto r = model::solve(std::move(init), c, N, dt, bound(Alow, Dlow), bound(Ahigh, Dhigh), cost);
    REQUIRE(r.p(Axis::X) == Approx(-0.5));
    REQUIRE(r.p(Axis::Y) == Approx(10));
    REQUIRE(r.psi == Approx(-0.0817101));
    REQUIRE(r.v == Approx(10.05));
    REQUIRE(r.current.angle == Approx(-0.436332));
    REQUIRE(r.current.throttle == Approx(1));
  }

  SECTION("50 iterations") {
    constexpr size_t iters = 50;
    State state(psi, v, xy, Control(0, 0));
    for (size_t i = 0; i < iters; ++i) {
      // TODO: fix this ugly syntax
      new(&state)State{model::solve(std::move(state), c, N, dt, bound(Alow, Dlow), bound(Ahigh, Dhigh), cost)};
    }
    REQUIRE(state.p(Axis::X) == Approx(21.1792));
    REQUIRE(state.p(Axis::Y) == Approx(-0.997072));
    REQUIRE(state.psi == Approx(0.000309515).epsilon(1e-4));
    REQUIRE(state.v == Approx(12.5));
    REQUIRE(state.current.angle == Approx(-0.0038108));
    REQUIRE(state.current.throttle == Approx(1));
  }
}
