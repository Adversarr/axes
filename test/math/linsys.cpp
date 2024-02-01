#include <doctest/doctest.h>

#include <entt/core/hashed_string.hpp>
#include <entt/locator/locator.hpp>

#include "axes/math/linsys/dense.hpp"

TEST_CASE("Refl") {
  using namespace ax::utils;
  using namespace ax::math;
  auto ldlt = reflect_enum<LinsysSolverKind>("LDLT");
  CHECK(ldlt.has_value());
  CHECK(ldlt.value() == LinsysSolverKind::kLDLT);

  auto partialLU = reflect_enum<LinsysSolverKind>("PartialLU");
  CHECK(partialLU.has_value());
  CHECK(partialLU.value() == LinsysSolverKind::kPartialLU);
}
