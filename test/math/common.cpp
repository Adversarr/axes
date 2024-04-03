#include "ax/math/functional.hpp"
#include "ax/utils/iota.hpp"
#include <doctest/doctest.h>

#include <ax/math/common.hpp>

using namespace ax;
using namespace ax::math;
TEST_CASE("math-types") {
  vec2i v2i{1, 2};
  CHECK(v2i.x() == 1);
  CHECK(v2i.y() == 2);
}

TEST_CASE("ones") {
  vec2i v2i = math::ones<2, 1, idx>();
  CHECK(v2i.x() == 1);
  CHECK(v2i.y() == 1);

  mat2r m2r = math::ones(2, 2);
  for (idx i = 0; i < 2; ++i) {
    for (idx j = 0; j < 2; ++j) {
      CHECK(m2r(i, j) == doctest::Approx(1.0));
    }
  }
}

TEST_CASE("zeros") {
  vec2i v2i = math::zeros<2, 1, idx>();
  CHECK(v2i.x() == 0);
  CHECK(v2i.y() == 0);

  mat2r m2r = math::zeros(2, 2);
  for (idx i = 0; i < 2; ++i) {
    for (idx j = 0; j < 2; ++j) {
      CHECK(m2r(i, j) == doctest::Approx(0.0));
    }
  }
}

TEST_CASE("linspace") {
  vec2r v2r = math::linspace(0.0, 1.0, 2);
  CHECK(v2r.x() == doctest::Approx(0.0));
  CHECK(v2r.y() == doctest::Approx(1.0));

  vec2r v2r2 = math::linspace(0.0, 0.0, 2);
  CHECK(v2r2.x() == doctest::Approx(0.0));
  CHECK(v2r2.y() == doctest::Approx(0.0));

  vec3r v3r3 = math::linspace<3>(4.0);
  CHECK(v3r3.x() == doctest::Approx(0.0));
  CHECK(v3r3.y() == doctest::Approx(2.0));
  CHECK(v3r3.z() == doctest::Approx(4.0));
}

TEST_CASE("eye") {
  mat2r m2r = math::eye(2);
  CHECK(m2r(0, 0) == doctest::Approx(1.0));
  CHECK(m2r(0, 1) == doctest::Approx(0.0));
  CHECK(m2r(1, 0) == doctest::Approx(0.0));
  CHECK(m2r(1, 1) == doctest::Approx(1.0));
}

TEST_CASE("diag") {
  vec2r v2r{1.0, 2.0};
  v2r.asDiagonal();
  mat2r m2r = math::diag(v2r);
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
  auto f = make_field<field3i>(2);
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
  auto iota1 = utils::multi_iota(2, 3, 4);
  auto it1 = iota1.begin();
  for (auto i: utils::iota(2)) {
    for (auto j: utils::iota(3)) {
      for (auto k: utils::iota(4)) {
        auto [ii, jj, kk] = *(it1++);
        CHECK_EQ(i, ii);
        CHECK_EQ(j, jj);
        CHECK_EQ(k, kk);
      }
    }
  }
  auto iota2 = utils::multi_iota(
    utils::ituple(0, 2), 
    utils::ituple(0, 3, 2),
    4);

  auto it2 = iota2.begin();
  for (auto i: utils::iota(0, 2)) {
    for (auto j: utils::iota(0, 3, 2)) {
      for (auto k: utils::iota(4)) {
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
  auto v3f0 = make_zeros<vec3f>();
  CHECK(v3f0 == vec3f::Zero());

  auto v3f1 = make_ones<vec3f>();
  CHECK(v3f1 == vec3f::Ones());
}