#include "ax/math/accessor.hpp"

#include <doctest/doctest.h>

#include <ax/math/shape.hpp>

#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/structure_binding.hpp"
#include "ax/math/views.hpp"

TEST_CASE("shape3d") {
  using namespace ax::math;
  auto s = make_shape(3, 4, 5);
  SUBCASE("structure binding") {
    auto [a, b, c] = s.Extent();
    CHECK_EQ(a, 3);
    CHECK_EQ(b, 4);
    CHECK_EQ(c, 5);
  }

  SUBCASE("ind2sub") {
    size_t ind = 4 * 5 * 2 + 5 * 3 + 4;
    auto [a, b, c] = s.Ind2Sub(ind);
    CHECK_EQ(a, 2);
    CHECK_EQ(b, 3);
    CHECK_EQ(c, 4);
  }

  SUBCASE("iterate over shape") {
    size_t ind = 0;
    for (auto [i, j, k] : iter(s)) {
      CHECK_EQ(s.Sub2Ind(i, j, k), ind);
      ind++;
    }
  }
}

TEST_CASE("shape1d") {
  using namespace ax::math;
  auto s = make_shape(8);
  size_t cnt = 0;
  for (auto ind : iter(s)) {
    CHECK_EQ(ind, cnt);
    cnt++;
  }
}

TEST_CASE("accessor3d_eigen") {
  using namespace ax::math;
  auto s = make_shape<ptrdiff_t>(3, 4, 5);
  RealField3 f(3, s.Size());
  auto fa = make_accessor(f, s);
  {
    for (auto [i, j, k] : iter(s)) {
      fa(i, j, k).x() = i;
      fa(i, j, k).y() = j;
      fa(i, j, k).z() = k;
    }

    for (ptrdiff_t ind = 0; ind < s.Size(); ind++) {
      auto [i, j, k] = s.Ind2Sub(ind);
      CHECK_EQ(fa(i, j, k).x(), i);
      CHECK_EQ(fa(i, j, k).y(), j);
      CHECK_EQ(fa(i, j, k).z(), k);

      CHECK_EQ(f.col(ind).x(), i);
      CHECK_EQ(f.col(ind).y(), j);
      CHECK_EQ(f.col(ind).z(), k);
    }
  }
  fa.Data().setZero();

  {  // enumerate
    for (auto [ijk, colvec] : enumerate(fa)) {
      auto [i, j, k] = ijk;
      colvec.x() = i;
      colvec.y() = j;
      colvec.z() = k;
    }

    for (ptrdiff_t ind = 0; ind < s.Size(); ind++) {
      auto [i, j, k] = s.Ind2Sub(ind);
      CHECK_EQ(fa(i, j, k).x(), i);
      CHECK_EQ(fa(i, j, k).y(), j);
      CHECK_EQ(fa(i, j, k).z(), k);

      CHECK_EQ(f.col(ind).x(), i);
      CHECK_EQ(f.col(ind).y(), j);
      CHECK_EQ(f.col(ind).z(), k);
    }
  }
}

TEST_CASE("accessor3d_stlvec") {
  using namespace ax::math;
  auto s = make_shape<size_t>(3, 4, 5);
  std::vector<RealVector3> f(s.Size());
  auto fa = make_accessor(f, s);
  {
    for (auto [i, j, k] : iter(s)) {
      fa(i, j, k).x() = i;
      fa(i, j, k).y() = j;
      fa(i, j, k).z() = k;
    }

    for (size_t ind = 0; ind < s.Size(); ind++) {
      auto [i, j, k] = s.Ind2Sub(ind);
      CHECK_EQ(fa(i, j, k).x(), i);
      CHECK_EQ(fa(i, j, k).y(), j);
      CHECK_EQ(fa(i, j, k).z(), k);

      CHECK_EQ(f[ind].x(), i);
      CHECK_EQ(f[ind].y(), j);
      CHECK_EQ(f[ind].z(), k);
    }
  }
  std::fill(fa.begin(), fa.end(), RealVector3::Zero());

  {  // enumerate
    for (auto [ijk, colvec] : enumerate(fa)) {
      auto [i, j, k] = ijk;
      colvec.x() = i;
      colvec.y() = j;
      colvec.z() = k;
    }

    for (size_t ind = 0; ind < s.Size(); ind++) {
      auto [i, j, k] = s.Ind2Sub(ind);
      CHECK_EQ(fa(i, j, k).x(), i);
      CHECK_EQ(fa(i, j, k).y(), j);
      CHECK_EQ(fa(i, j, k).z(), k);

      CHECK_EQ(f[ind].x(), i);
      CHECK_EQ(f[ind].y(), j);
      CHECK_EQ(f[ind].z(), k);
    }
  }
}

TEST_CASE("accessor1d3d") {
  using namespace ax;
  using namespace ax::math;
  RealField3 f(3, 8);

  auto a3 = make_accessor(f, make_shape(2, 2, 2));
  auto a1 = make_accessor(f);

  {  // Their underlying data is same.
    for (auto [ijk, v] : enumerate(a3)) {
      v = to_vec(ijk).cast<real>();
    }

    for (auto [ind, v] : enumerate(a1)) {
      auto [i, j, k] = a3.GetShape().Ind2Sub(ind);
      CHECK_EQ(v.x(), i);
      CHECK_EQ(v.y(), j);
      CHECK_EQ(v.z(), k);
    }
  }

  {  // Inversely
  }
}

TEST_CASE("accessor_span1d") {
  using namespace ax;
  using namespace ax::math;
  RealField1 f(8);
  real* data = f.data();
  size_t size = static_cast<size_t>(f.size());
  Span chk(data, size);

  auto acc = make_accessor(chk);
  for (size_t i = 0; i < size; i++) {
    acc(i) = i;
  }

  // assert that the data is same
  for (Index i = 0; i < f.size(); ++i) {
    CHECK_EQ(f[i], i);
  }
}

TEST_CASE("accessor_span3d") {
  using namespace ax;
  using namespace ax::math;
  std::vector<RealVector3> f(8);
  auto* data = f.data();
  size_t size = f.size();
  Span chk(data, size);

  auto acc = make_accessor(chk, make_shape(2, 2, 2));
  for (auto [ijk, val] : enumerate(acc)) {
    val = to_vec(ijk).cast<real>();
  }

  // assert that the data is same
  for (size_t i = 0; i < f.size(); ++i) {
    auto ijk = acc.GetShape().Ind2Sub(i);
    CHECK_EQ(f[i].x(), ijk[0]);
    CHECK_EQ(f[i].y(), ijk[1]);
    CHECK_EQ(f[i].z(), ijk[2]);
  }
}

using namespace ax;
using namespace ax::math;

TEST_CASE("const_accessor2d") {
  std::vector<RealVector3> f(8);
  auto shape = make_shape(2, 4);
  auto acc = make_accessor(f, shape);

  for (auto [i, j] : iter(shape)) {
    acc(i, j) = RealVector3(static_cast<real>(i), static_cast<real>(j), 0);
  }

  auto const_acc = make_accessor(std::as_const(f), shape);
  for (auto [ij, v] : enumerate(const_acc)) {
    auto [i, j] = ij;
    CHECK_EQ(v.x(), static_cast<real>(i));
    CHECK_EQ(v.y(), static_cast<real>(j));
    CHECK_EQ(v.z(), 0);
  }
}

template <typename Rng, typename F> void for_each(Rng&& r, F fun) {
  for (auto&& item : r) {
    if constexpr (std::is_invocable_v<F, decltype(item)>) {
      fun(item);
    } else {
      ranges::tuple_apply(fun, item);
    }
  }
}

TEST_CASE("for_each3d") {
  std::vector<RealVector3> f(8);
  auto shape = make_shape(2, 2, 2);
  auto acc = make_accessor(f, shape);

  for_each(enumerate(acc), [](auto && ijk, auto & v) { v = to_vec(ijk).template cast<real>(); });

  for (auto [i, j, k] : iter(shape)) {
    auto ind = shape.Sub2Ind(i, j, k);
    CHECK_EQ(f[ind].x(), i);
    CHECK_EQ(f[ind].y(), j);
    CHECK_EQ(f[ind].z(), k);
  }
}