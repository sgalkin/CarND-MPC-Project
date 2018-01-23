#include "catch.hpp"
#include "model.h"

TEST_CASE("Model") {
  size_t N = 25;
  double dt = 0.05;
  size_t iters = 50;

  Eigen::VectorXd ptsx(2);
  Eigen::VectorXd ptsy(2);
  ptsx << -100, 100;
  ptsy << -1, -1;

  // The polynomial is fitted to a straight line so a polynomial with
  // order 1 is sufficient.
  Polynomial<1> c(ptsx, ptsy);

  // NOTE: free feel to play around with these
  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  State s(psi, v, (Eigen::Vector2d() << x, y).finished(), Control(0,0), Eigen::MatrixXd());
  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;

    auto vars = Model<1>(s, c, N, dt).solve();
  }
}
