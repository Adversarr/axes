#include "ax/graph/payload.hpp"
#include <doctest/doctest.h>

struct Noisy {
  Noisy() { std::cout << "Noisy(): " << this << std::endl; }
  ~Noisy() { std::cout << "~Noisy(): " << this << std::endl; }
};

using namespace ax;
using namespace graph;