#include "io.h"
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

/*
{"psi":3.733651,"psi_unity":4.12033,
    "ptsx":[-32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717],
    "ptsy":[113.361,105.941,92.88499,78.73102,65.34102,50.57938],
    "speed":0,
    "steering_angle":0,
    "throttle":0,
    "x":-40.62,
    "y":108.73}
*/

State Json::operator()(std::string message) {
  auto j = extract(std::move(message));
  std::cerr << j.dump() << std::endl;
  return State();
}

std::string Json::operator()(Control /*control*/) {
  return "{}";
}
