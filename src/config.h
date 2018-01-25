#pragma once

#include <iostream>
#include <iomanip>
#include <chrono>
#include "ProgramOptions.hxx"
  
struct Config {
  const uint16_t port;
  const std::chrono::milliseconds delay;

  const size_t depth;
  const std::chrono::duration<double> dt;

  Config(int argc, char* argv[])
    : Config([](int argc, char* argv[]) {
        auto parser = Config::parser();
        if(!parser(argc, argv)) {
          std::cerr << parser << std::endl;
          throw std::runtime_error("invalid arguments");
        }
        return parser;
      }(argc, argv))
  {}
    
  explicit Config(po::parser p)
    : port{uint16_t(p["port"].get().u32)}
    , delay{p["delay"].get().u32}
    , depth{p["depth"].get().u32}
    , dt{
      std::chrono::duration_cast<
        std::chrono::duration<double>>(
          std::chrono::milliseconds(p["dt"].get().u32))
        }
  {}

  static po::parser parser() {
    constexpr uint16_t port = 4567;

    constexpr std::chrono::milliseconds delay{100};

    constexpr size_t depth{8};
    constexpr std::chrono::milliseconds dt{125};

    po::parser p;
    p["help"].abbreviation('h')
      .description("print this help screen")
      .callback([&p]{ std::cerr << p << '\n'; exit(1); });
    p["port"].abbreviation('p').type(po::u32).fallback(port)
      .description("Port to use (default: " + std::to_string(port) + ")");
    p["depth"].abbreviation('N').type(po::u32).fallback(depth)
      .description("Prediction depth (default: " + std::to_string(depth) + ")");
    p["dt"].abbreviation('t').type(po::u32).fallback(dt.count())
      .description("Prediction time step, ms (default: " + std::to_string(dt.count()) + ")");
    p["delay"].abbreviation('d').type(po::u32).fallback(delay.count())
      .description("Control delay, ms (default: " + std::to_string(delay.count()) + ")");
    return p;
  }
};

template<typename OS>
OS& operator<< (OS& os, const Config& c) {
  os << "{"
     << "\"Port\":" << c.port << ","
     << "\"Delay\":" << c.delay.count() << ","
     << "\"Depth\":" << c.depth << ","
     << "\"dt\":" << std::setprecision(6) << c.dt.count()
     << "}";
  return os;
}
