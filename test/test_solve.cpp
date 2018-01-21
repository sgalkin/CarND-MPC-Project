#include "catch.hpp"
#include "solve.h"
#include "solve_traits.h"

template<typename T> using V = std::vector<T>;

TEST_CASE("Solve") {
  using ST = solve_traits<V, double>;
  
  auto cost = [](const ST::ADvector& x) { return x[0]; };
 
  SECTION("Unconstrained") {
    struct Constraint {
      using Dvector = ST::Dvector;
      using ADvector = ST::ADvector;
      Dvector upper_bound() const { return Dvector(); }
      Dvector lower_bound() const { return Dvector(); }
      ADvector operator()(const ADvector&) const { return ADvector(); }
    } constraint;
    struct Variable {
      using Dvector = ST::Dvector;
      Dvector upper_bound() const { return Dvector(1, ST::unbounded); }
      Dvector lower_bound() const { return Dvector(1, -ST::unbounded); }
      Dvector operator()() const { return Dvector(1, 0); }
    } variable;
    auto solution = solve(cost, constraint, variable);
    REQUIRE(solution.status != decltype(solution)::success);
  }
  
  SECTION("Constrainted") {
    struct Constraint {
      using Dvector = ST::Dvector;
      using ADvector = ST::ADvector;
      Dvector upper_bound() const { return Dvector(1, 1); }
      Dvector lower_bound() const { return Dvector(1, 1); }
      ADvector operator()(const ADvector& x) const {
        return ADvector(1, x[0] + 8);
      }
    } constraint;
    struct Variable {
      using Dvector = ST::Dvector;
      Dvector upper_bound() const { return Dvector(1, 10); }
      Dvector lower_bound() const { return Dvector(1, -10); }
      Dvector operator()() const { return Dvector(1, 0); }
    } variable;
    auto solution = solve(cost, constraint, variable);
    REQUIRE(solution.status == decltype(solution)::success);
    REQUIRE(solution.x[0] == Approx(-7));
  }
}
