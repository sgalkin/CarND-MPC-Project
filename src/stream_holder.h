#pragma once

#include <fstream>
#include <iostream>

class StreamHolder {
public:
  explicit StreamHolder(std::basic_ostream<char>& stream)
    : stream_(stream)
  {}
  
  explicit StreamHolder(const std::string& name)
    : file_{new std::ofstream(name.c_str())}
    , stream_(*file_)
  {}

  std::basic_ostream<char>& operator*() { return stream_; }
  
private:
  std::unique_ptr<std::ofstream> file_;
  std::basic_ostream<char>& stream_;
};
  
