#include "mpc.h"

#include <Eigen/Core>

#include "model.h"
#include "control.h"
#include "polynomial.h"
#include "util.h"

namespace {
constexpr size_t degree{3};

constexpr double Alb = -0.05;
constexpr double Aub = 1;

constexpr double Slb = -18.*M_PI/180.;
constexpr double Sub = -Slb;

constexpr double Vref = 80;

class CsvOutput {
public:
  void operator()(model::Solve::Dvector::value_type cost,
                  model::Index idx,
                  const model::Solve::Dvector& solution) {
    std::cerr << ++count_ << "," << cost << ","
              << solution[idx(model::CTE, 1)] << ","
              << solution[idx(model::EPSI, 1)] << ","
              << solution[idx(model::V, 1)] << ","
              << solution[idx(model::PSI, 1)] << ","
              << solution[idx(model::A, 0)] << ","
              << solution[idx(model::S, 0)] << "\n";
  };
private:
  static size_t count_;
};

size_t CsvOutput::count_ = 0;
  
model::Solve::ADvector::value_type cost(model::Index idx, const model::Solve::ADvector& x) {
  model::Solve::ADvector::value_type v = 0;
  model::Solve::ADvector::value_type mcte = 0;
  model::Solve::ADvector::value_type mepsi = 0;
  
  for (size_t t = 0; t < idx.depth; ++t) {
    auto cte = pow(x[idx(model::CTE, t)], 2);
    if(mcte < cte) mcte = cte;
    auto epsi = pow(x[idx(model::EPSI, t)], 2);
    if(mepsi < epsi) mepsi = epsi;
    v += cte;
    v += epsi;
    v += 128*pow((x[idx(model::V, t)] - Vref)/Vref, 2);
  }
  
  // Penalty on maximum deviation from reference
  v += 128*idx.depth*mcte;
  v += 64*idx.depth*mepsi;
  
  // Minimize the use of actuators.
  for (size_t t = 0; t < idx.depth - 1; ++t) {
    v += pow(x[idx(model::A, t)]/(Aub-Alb), 2);
    v += 64*pow(x[idx(model::S, t)]/(Sub-Slb), 2);
  }
  
  // Minimize the value gap between sequential actuations.
  for (size_t t = 0; t < idx.depth - 2; t++) {
    v += pow((x[idx(model::A, t + 1)] - x[idx(model::A, t)])/(Aub-Alb), 2);
    v += 630*pow((x[idx(model::S, t + 1)] - x[idx(model::S, t)])/(Sub-Slb), 2);
  }
  return v;  
}
}


Control MPC::operator()(State s) {
  auto poly = Polynomial<degree>(std::move(s.wp));
  
  Eigen::MatrixXd waypoints(10, 2);
  waypoints.col(Axis::X) = Eigen::VectorXd::LinSpaced(10, 5, 80);
  waypoints.col(Axis::Y) = poly(waypoints.col(Axis::X));

  auto bind = [](model::Solve::Dvector::value_type a,
                 model::Solve::Dvector::value_type s) {
    return [a, s](model::Index idx, model::Solve::Dvector v) {
      for(size_t i = 0; i < idx.depth - 1; ++i) {
        v[idx(model::A, i)] = a;
        v[idx(model::S, i)] = s;
      }
      return v;
    };
  };

  auto r = model::solve(std::move(s), std::move(poly), N, dt.count(),
                        cost, bind(Alb, Slb), bind(Aub, Sub), CsvOutput{});

  return Control(r.current.angle, r.current.throttle,
                 std::move(r.current.prediction),
                 std::move(waypoints));
}
