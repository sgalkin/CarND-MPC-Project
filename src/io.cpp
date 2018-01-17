#include "io.h"

#include <Eigen/Core>

#include "json.hpp"
#include "state.h"
#include "control.h"

using json = nlohmann::json;

namespace {
json extract(std::string message) {
  auto j = json::parse(std::move(message));
  if (j[0].get<std::string>() != "telemetry") {
    throw std::runtime_error("Unexpected event type");
  }
  return j[1];
}
}

State Json::operator()(std::string message) {
  auto j = extract(std::move(message));
  if(j["ptsx"].size() != j["ptsy"].size())
    throw std::runtime_error("ptsx/ptsy size mismatch");

  double psim = j["psi"].get<double>();
  double psiu = j["psi_unity"].get<double>();
  Eigen::MatrixXd wp(j["ptsx"].size(), 2);  
  wp.col(0) = Eigen::Map<Eigen::VectorXd>(j["ptsx"].get<std::vector<double>>().data(),
                                          j["ptsx"].size());
  wp.col(1) = Eigen::Map<Eigen::VectorXd>(j["ptsy"].get<std::vector<double>>().data(),
                                          j["ptsy"].size());
  double speed = j["speed"].get<double>();
  double angle = j["steering_angle"].get<double>();
  double throttle = j["throttle"].get<double>();
  Eigen::Vector2d p(j["x"].get<double>(), j["y"].get<double>());
  
  return State(psim, psiu, std::move(wp), speed, angle, throttle, std::move(p));
}

std::string Json::operator()(Control c) {
  json j;
  j["steering_angle"] = c.angle;
  j["throttle"] = c.throttle;

  j["mpc_x"] = std::vector<double>(c.prediction.col(0).data(),
                                   c.prediction.col(0).data() + c.prediction.col(0).size());
  j["mpc_y"] = std::vector<double>(c.prediction.col(1).data(),
                                   c.prediction.col(1).data() + c.prediction.col(1).size());

  j["next_x"] = std::vector<double>(c.wp.col(0).data(),
                                    c.wp.col(0).data() + c.wp.col(0).size());
  j["next_y"] = std::vector<double>(c.wp.col(1).data(),
                                    c.wp.col(1).data() + c.wp.col(1).size());
  std::cerr << j.dump() << std::endl;
  return j.dump();
}
