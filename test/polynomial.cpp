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
}
