#include "io.h"

#include <cmath>
#include <Eigen/Core>

#include "json.hpp"
#include "model.h"
#include "control.h"

namespace {
  static const size_t _coverage = 0;
}

using json = nlohmann::json;

namespace {
json extract(std::string message) {
  auto j = json::parse(std::move(message));
  if (j[0].get<std::string>() != "telemetry") {
    throw std::runtime_error("Unexpected event type");
  }
  return j[1];
}

template<typename T>
std::vector<typename T::Scalar> to_vector(const T& src) {
  static_assert(T::RowsAtCompileTime == 1 || T::ColsAtCompileTime == 1, "src is not a vector");
  assert(src.rows() == 1 || src.cols() == 1);
  return std::vector<typename T::Scalar>{ src.data(), src.data() + src.size() };
}

Eigen::VectorXd to_vector(std::vector<double>&& v) {
  return Eigen::Map<Eigen::VectorXd>(v.data(), v.size());
}
  
/*
double radian(double degree) {
  return degree*M_PI/180.;
}
*/
/*
double degree(double radian) {
  return radian*180./M_PI;
}
*/
}

Model Json::operator()(std::string message) {
  auto j = extract(std::move(message));
  if(j["ptsx"].size() != j["ptsy"].size())
    throw std::runtime_error("ptsx/ptsy size mismatch");

  double psim = j["psi"].get<double>();
//  double psiu = j["psi_unity"].get<double>();
  Eigen::MatrixXd wp(j["ptsx"].size(), 2);
  wp.col(State::X) = to_vector(j["ptsx"]);
  wp.col(State::Y) = to_vector(j["ptsy"]);

  double speed = j["speed"].get<double>();
  double angle = j["steering_angle"].get<double>();
  double throttle = j["throttle"].get<double>();
  Eigen::Vector2d p(j["x"].get<double>(), j["y"].get<double>());
  
  return Model(std::move(wp),
               State(psim, /*psiu,*/ speed, std::move(p)),
               Control(angle, throttle));
}

std::string Json::operator()(Control c) {
  json j;
  j["steering_angle"] = c.angle;
  j["throttle"] = c.throttle;

  j["mpc_x"] = to_vector(c.prediction.col(State::X));
  j["mpc_y"] = to_vector(c.prediction.col(State::Y));
   
  j["next_x"] = to_vector(c.wp.col(State::X));
  j["next_y"] = to_vector(c.wp.col(State::Y));
  return j.dump();
}
