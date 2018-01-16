#pragma once

#include <string>

template<typename Protocol, typename Reader, typename Writer, typename Processor>
class Application {
public:
  std::string ProcessMessage(std::string message) {
    auto payload = Protocol::getPayload(std::move(message));
    if(payload.empty()) return Protocol::formatResponse();

    auto state = reader_(std::move(payload));
    auto control = processor_(std::move(state));
    auto reply = writer_(std::move(control));
    return Protocol::formatResponse(std::move(reply));
  }

private:
  Reader reader_;
  Writer writer_;
  Processor processor_;
};
