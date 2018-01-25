#pragma once

#include <iostream>
#include <iomanip>
#include "model_index.h"

namespace model {
namespace output {
using Output = std::function<void(Solve::Dvector::value_type, Index, const Solve::Dvector&)>;

inline void No(Solve::Dvector::value_type, Index, const Solve::Dvector&) {}

template<typename OS>
class CSV {
public:
  explicit CSV(OS& out)
    : out_(out)
  {}
  
  void operator()(Solve::Dvector::value_type cost, Index idx, const Solve::Dvector& solution) const {
    out_ << ++count_ << "," << cost << ","
         << solution[idx(CTE, 1)] << ","
         << solution[idx(EPSI, 1)] << ","
         << solution[idx(V, 1)] << ","
         << solution[idx(PSI, 1)] << ","
         << solution[idx(A, 0)] << ","
         << solution[idx(S, 0)] << "\n";
  };
  
private:
  // TODO: this counter is ugly now
  mutable size_t count_{0};
  OS& out_;
};

template<typename OS>
CSV<OS> csv(OS& output) { return CSV<OS>{output}; }

template<typename OS>
class Brief {
public:
  explicit Brief(OS& out)
    : out_(out)
  {}
  
  void operator()(Solve::Dvector::value_type cost, Index idx, const Solve::Dvector& solution) const {
    out_ << "Cost: "
         << std::setw(12) << std::setprecision(6) << std::fixed
         << cost
         << " S: "
         << std::setw(7) << std::setprecision(4) << std::fixed
         << solution[idx(S, 0)]
         << " A: "
         << std::setw(5) << std::setprecision(2) << std::fixed
         << solution[idx(A, 0)]
         << "\n";
  }
private:
  OS& out_;
};

template<typename OS=decltype(std::cout)>
Brief<OS> brief(OS& output=std::cout) { return Brief<OS>{output}; }
  
}
}
