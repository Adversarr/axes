#include "ax/math/functional.hpp"
#include "ax/utils/ndrange.hpp"
#include <doctest/doctest.h>

#include <ax/math/common.hpp>

using namespace ax;
using namespace ax::math;
TEST_CASE("math-types") {
  IndexVec2 v2i{1, 2};
  CHECK(v2i.x() == 1);
  CHECK(v2i.y() == 2);
}

TEST_CASE("ones") {
  IndexVec2 v2i = math::ones<2, 1, Index>();
  CHECK(v2i.x() == 1);
  CHECK(v2i.y() == 1);

  RealMatrix2 m2r = math::ones(2, 2);
  for (Index i = 0; i < 2; ++i) {
    for (Index j = 0; j < 2; ++j) {
      CHECK(m2r(i, j) == doctest::Approx(1.0));
    }
  }
}

TEST_CASE("zeros") {
  IndexVec2 v2i = math::zeros<2, 1, Index>();
  CHECK(v2i.x() == 0);
  CHECK(v2i.y() == 0);

  RealMatrix2 m2r = math::zeros(2, 2);
  for (Index i = 0; i < 2; ++i) {
    for (Index j = 0; j < 2; ++j) {
      CHECK(m2r(i, j) == doctest::Approx(0.0));
    }
  }
}

TEST_CASE("linspace") {
  RealVector2 v2r = math::linspace(0.0, 1.0, 2);
  CHECK(v2r.x() == doctest::Approx(0.0));
  CHECK(v2r.y() == doctest::Approx(1.0));

  RealVector2 v2r2 = math::linspace(0.0, 0.0, 2);
  CHECK(v2r2.x() == doctest::Approx(0.0));
  CHECK(v2r2.y() == doctest::Approx(0.0));

  RealVector3 v3r3 = math::linspace<3>(4.0);
  CHECK(v3r3.x() == doctest::Approx(0.0));
  CHECK(v3r3.y() == doctest::Approx(2.0));
  CHECK(v3r3.z() == doctest::Approx(4.0));
}

TEST_CASE("eye") {
  RealMatrix2 m2r = math::eye(2);
  CHECK(m2r(0, 0) == doctest::Approx(1.0));
  CHECK(m2r(0, 1) == doctest::Approx(0.0));
  CHECK(m2r(1, 0) == doctest::Approx(0.0));
  CHECK(m2r(1, 1) == doctest::Approx(1.0));
}

TEST_CASE("diag") {
  RealVector2 v2r{1.0, 2.0};
  v2r.asDiagonal();
  RealMatrix2 m2r = math::diag(v2r);
  CHECK(m2r(0, 0) == doctest::Approx(1.0));
  CHECK(m2r(0, 1) == doctest::Approx(0.0));
  CHECK(m2r(1, 0) == doctest::Approx(0.0));
  CHECK(m2r(1, 1) == doctest::Approx(2.0));

  v2r.setZero();
  v2r = math::diag(m2r);
  CHECK(v2r.x() == doctest::Approx(1.0));
  CHECK(v2r.y() == doctest::Approx(2.0));
}

TEST_CASE("iter") {
  auto f = make_field<IndexField3>(2);
  for (auto v3i: each(f)) {
    v3i.setOnes();
  }

  for (auto v3i: each(f)) {
    CHECK(v3i.x() == 1);
    CHECK(v3i.y() == 1);
    CHECK(v3i.z() == 1);
  }

  for (auto v3i: each(f)) {
    v3i.setZero();
  }

  for (auto v3i: each(f)) {
    CHECK(v3i.x() == 0);
    CHECK(v3i.y() == 0);
    CHECK(v3i.z() == 0);
  }
}

TEST_CASE("mult-iota") {
  using namespace ax;
  auto iota1 = utils::ndrange<Index>(2, 3, 4);
  auto it1 = iota1.begin();
  for (auto i: utils::range(2)) {
    for (auto j: utils::range(3)) {
      for (auto k: utils::range(4)) {
        auto [ii, jj, kk] = *(it1++);
        CHECK_EQ(i, ii);
        CHECK_EQ(j, jj);
        CHECK_EQ(k, kk);
      }
    }
  }
  auto iota2 = utils::ndrange<Index>(
    utils::make_index_tuple(0, 2),
    utils::make_index_tuple(0, 3, 2),
    4);

  auto it2 = iota2.begin();
  for (auto i: utils::range(0, 2)) {
    for (auto j: utils::range(0, 3, 2)) {
      for (auto k: utils::range(4)) {
        auto [ii, jj, kk] = *(it2++);
        CHECK_EQ(i, ii);
        CHECK_EQ(j, jj);
        CHECK_EQ(k, kk);
      }
    }
  }
}

TEST_CASE("constructor") {
  using namespace ax::math;
  auto f0 = make_zeros<float>();
  CHECK(f0 == 0.0f);
  auto f1 = make_ones<float>();
  CHECK(f1 == 1.0f);
  auto v3f0 = make_zeros<FloatVector3>();
  CHECK(v3f0 == FloatVector3::Zero());

  auto v3f1 = make_ones<FloatVector3>();
  CHECK(v3f1 == FloatVector3::Ones());
}