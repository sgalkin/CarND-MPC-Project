#pragma once

#include <string>

template<typename Protocol, typename Reader, typename Writer, typename Pipeline>
class Application {
public:
  explicit Application(Pipeline pipeline) :
    pipeline_(std::move(pipeline))
  {}
  
  std::string ProcessMessage(std::string message) {
    auto payload = Protocol::getPayload(std::move(message));
    if(payload.empty()) return Protocol::formatResponse();

    auto model = reader_(std::move(payload));
    auto control = pipeline_(std::move(model));
    auto reply = writer_(std::move(control));
    return Protocol::formatResponse(std::move(reply));
  }

private:
  Reader reader_;
  Writer writer_;
  Pipeline pipeline_;
};
