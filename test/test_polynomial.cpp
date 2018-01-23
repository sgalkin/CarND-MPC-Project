#include "catch.hpp"
#include "polynomial.h"

TEST_CASE("Polynomial") {
  SECTION("Coefficient construction") {
    Polynomial<1> x((Eigen::Vector2d() << -1, 1).finished());
    REQUIRE(x(0.) == Approx(-1));
    REQUIRE(x(1.) == Approx(0));
  }

  SECTION("Fit construction x,y") {
    constexpr size_t n = 2;
    constexpr size_t m = 16;
    
    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(m, 0, m-1);
    Eigen::VectorXd y = (x + Eigen::VectorXd::Constant(m, 3)).array().pow(n);
    REQUIRE(x.size() == m);
    REQUIRE(y.size() == m);
    
    auto px = Polynomial<n>(x, y)(x);
    for(int i = 0; i < px.rows(); ++i)
      REQUIRE(px(i, 0) == Approx(y(i, 0)));
  }
  
  SECTION("Fit construction xy") {
    constexpr size_t n = 2;
    constexpr size_t m = 16;
    
    Eigen::Matrix<double, 8, 2> xy;
    for(int i = 0; i < 8; ++i) {
      xy(i, 0) = i;
      xy(i, 1) = (i + 3)*(i + 3);
    }
    auto px = Polynomial<n>(xy)(xy.col(0));
    for(int i = 0; i < px.rows(); ++i)
      REQUIRE(px(i, 0) == Approx(xy(i, 1)));
  }

  SECTION("Evaluate static") {
    auto xy = (Eigen::Matrix<double, 8, 1>() << 1, 2, 3, 4, 5, 6, 7, 8).finished();
    auto p = Polynomial<1>{(Eigen::Matrix<double, 2, 1>() << 0, 1).finished()};
    REQUIRE(p(xy) == xy);
  }

  SECTION("Evaluate dynamic") {
    auto xy = (Eigen::VectorXd(8) << 1, 2, 3, 4, 5, 6, 7, 8).finished();
    auto p = Polynomial<1>{(Eigen::Matrix<double, 2, 1>() << 0, 1).finished()};
    REQUIRE(p(xy) == xy);
  }

  SECTION("Evaluate scalar") {
    auto xy = double{42};
    auto p = Polynomial<1>{(Eigen::Matrix<double, 2, 1>() << 0, 1).finished()};
    REQUIRE(p(xy) == xy);
  }
  
  SECTION("Derive") {
    constexpr size_t n = 3;
    Polynomial<n>::C c; c << 1, 2, 3, 4; // 1 + 2x + 3x^2 + 4x^3
    auto d = derive(Polynomial<n>(std::move(c)));
    REQUIRE(d.c == (Polynomial<n-1>::C() << 2, 6, 12).finished());
  }

  SECTION("Derive Constant") {
    constexpr size_t n = 0;
    Polynomial<n>::C c; c << 4; // 4
    auto d = derive(Polynomial<n>(std::move(c)));
    REQUIRE(d.c == (Polynomial<n>::C() << 0).finished());
  }
}
