#include <doctest/doctest.h>

#include <ax/geometry/measure.hpp>

using namespace ax;
using namespace ax::math;
using namespace ax::geo;

TEST_CASE("Measure Simplex") {
  Simplex2 s1{{
    RealVector2{0, 0},
    RealVector2{1, 0},
    RealVector2{0, 1}
  }};

  CHECK(measure(s1) == doctest::Approx(0.5));
}
