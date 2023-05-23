#include <doctest/doctest.h>

#include <axes/core/math/field.hpp>
#include <axes/core/math/indexer.hpp>
#include <iostream>

using namespace axes;
TEST_CASE("1d-ndrange") {
  NdRange<1> r1d({2});
  CHECK_EQ(r1d.dims_[0], 2);

  auto first = r1d.begin();

  for (auto i = 0; i < 2; ++i) {
    CHECK_EQ((*first)[0], i);
    CHECK((first != r1d.end()));
    ++first;
  }
  CHECK(!(first != r1d.end()));
}

TEST_CASE("2d-ndrange") {
  NdRange<2> r2d({2, 3});
  auto first = r2d.begin();

  for (auto i = 0; i < 2; ++i) {
    for (auto j = 0; j < 3; ++j) {
      CHECK_EQ((*first)[0], i);
      CHECK_EQ((*first)[1], j);
      CHECK((first != r2d.end()));
      ++first;
    }
  }
  CHECK(!(first != r2d.end()));
}

TEST_CASE("Field Basic") {
  Field<Vector3<>> field;
  field.Resize(3);
  auto view = field.CreateView();

  for (auto [i, v] : view) {
    v.setConstant(1);
  }

  for (const auto& v : field) {
    CHECK_EQ(3, v.sum());
  }
}

TEST_CASE("ndrangeIndexer") {
  NdRangeIndexer<3> indexer(3, 4, 5);
  CHECK(indexer.Size() == 60);
  CHECK(indexer(0, 0, 0) == 0);
  CHECK(indexer(0, 1, 0) == 5);
  CHECK(indexer(1, 1, 1) == 26);
  CHECK(!indexer.IsValid(std::array<size_t, 3>{3, 2, 1}));
  CHECK(indexer.IsValid(std::array<size_t, 3>{2, 2, 1}));

  auto it = NdRangeIndexer<2>(3, 2).Iterate().begin();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      CHECK_EQ((*it)[0], i);
      CHECK_EQ((*it)[1], j);
      ++it;
    }
  }
}
