#include <doctest/doctest.h>

#include <axes/geometry/measure.hpp>

using namespace ax;
using namespace ax::math;
using namespace ax::geo;

TEST_CASE("Measure Simplex") {
  Simplex2 s1{{
    vec2r{0, 0},
    vec2r{1, 0},
    vec2r{0, 1}
  }};

  CHECK(measure(s1) == doctest::Approx(0.5));
}
