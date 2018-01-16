#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <chrono>

#include <uWS/uWS.h>
#include "ProgramOptions.hxx"

#include "application.h"
#include "protocol.h"
#include "io.h"
#include "processor.h"
#include "compose.h"

namespace {
using WSApplication = Application<WSProtocol, Json, Json, Count>;

constexpr uint16_t port = 4567;
constexpr std::chrono::milliseconds delay(100);


po::parser parser() {
  po::parser p;
  p["help"].abbreviation('h').description("print this help screen")
    .callback([&p]{ std::cerr << p << '\n'; exit(1); });
  return p;
}

void run(po::parser /*p*/) {
//  auto config = ::config(p);
  std::unique_ptr<WSApplication> app;

  uWS::Hub h;
  h.onMessage([&app](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    try {
      auto message = std::string(data, length);
      auto response = app->ProcessMessage(std::move(message));

      // Latency
      // The purpose is to mimic real driving conditions where
      // the car does actuate the commands instantly.
      //
      // Feel free to play around with this value but should be to drive
      // around the track with 100ms latency.
      //
      // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
      // SUBMITTING.
      std::this_thread::sleep_for(delay);
      ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
    } catch(std::runtime_error& e) {
      std::cerr << "Error while processing message: " << e.what() << std::endl;
    }
  });

  h.onConnection([&app/*, config*/](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    app.reset(new WSApplication(/*config*/));
  });

  h.onDisconnection([&app](uWS::WebSocket<uWS::SERVER>, int, char*, size_t) {
    app.reset();
  });

  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
    h.run();
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
  }
}
}

int main(int argc, char* argv[]) {
  auto parser = ::parser();
  if(!parser(argc, argv)) {
    std::cerr << parser << std::endl;
    return -1;
  }
  run(std::move(parser));
  return 0;
}
