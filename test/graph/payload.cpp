#include "ax/graph/payload.hpp"
#include <doctest/doctest.h>

struct Noisy {
  Noisy() { std::cout << "Noisy(): " << this << std::endl; }
  ~Noisy() { std::cout << "~Noisy(): " << this << std::endl; }
};

using namespace ax;
using namespace graph;

TEST_CASE("payload creation") {
  auto noisy_payload = make_payload<Noisy>();
}

TEST_CASE("Registry") {
  PayloadTypeRegistry reg;
  reg.Register<int, int>(1);
  reg.Register<float, int>(2);
  reg.Register<float, float>(3);
  reg.Register<double, double>(4);
  reg.Register<int, float>(5);

  CHECK_EQ(reg.Get<int>().size(), 2);
  CHECK_EQ(*(reg.Get<int, int>()), 1);
  CHECK_EQ(*(reg.Get<float, int>()), 2);
  CHECK_EQ(*(reg.Get<float, float>()), 3);
  CHECK_EQ(*(reg.Get<double, double>()), 4);
  CHECK_EQ(*(reg.Get<int, float>()), 5);
}