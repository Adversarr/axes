#pragma once
#include "ax/core/excepts.hpp"

namespace ax::math {

class FailedPrefactorError : public std::exception {
public:
  virtual const char* what() const final { return "Failed to prefactor the matrix"; }
};

class FailedSolveError : public std::exception {
public:
  virtual const char* what() const final { return "Failed to solve the linsys."; }
};

}