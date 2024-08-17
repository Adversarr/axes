#include <doctest/doctest.h>
#include "ax/xpbd/constraint_map.hpp"
using namespace ax::xpbd;

TEST_CASE("constraint_map") {
  ConstraintMap map;

  CHECK(map.empty());
  CHECK(map.begin() == map.end());

  map.emplace_back(1, 2, 3);
  CHECK(map.Entries().size() == 1);
  CHECK(map[0][0] == 1);
  CHECK(map[0][1] == 2);
  CHECK(map[0][2] == 3);

  map.emplace_back(4, 5, 6);
  CHECK(map.Entries().size() == 2);
  CHECK(map[1][0] == 4);
  CHECK(map[1][1] == 5);
  CHECK(map[1][2] == 6);

  map.emplace_back(7, 8);
  CHECK(map.Entries().size() == 3);
  CHECK(map[2][0] == 7);
  CHECK(map[2][1] == 8);

  CHECK(map.Mapping().size() == 8);
}

#include "ax/xpbd/constraint_map.hpp"

TEST_CASE("ConstraintMap tests") {
  using namespace ax::xpbd;

  // Test emplace_back() function
  SUBCASE("emplace_back()") {
    ConstraintMap map;
    map.emplace_back(1, 2, 3);
    map.emplace_back(4, 5, 6);

    CHECK(map.Mapping() == std::vector<ax::Index>{1, 2, 3, 4, 5, 6});
    CHECK(map.Entries() == std::vector<size_t>{0, 3});
  }

  // Test reserve() function
  SUBCASE("reserve()") {
    ConstraintMap map;
    map.reserve(2, 3);

    CHECK(map.Mapping().capacity() >= 6);
    CHECK(map.Entries().capacity() >= 2);
  }

  // Test ConstVisitor
  SUBCASE("ConstVisitor") {
    ConstraintMap map;
    map.emplace_back(1, 2, 3);
    map.emplace_back(4, 5, 6);

    ConstraintMap::ConstVisitor visitor = map[0];

    CHECK(visitor.size() == 3);
    CHECK(visitor[0] == 1);
    CHECK(visitor[1] == 2);
    CHECK(visitor[2] == 3);
    CHECK_THROWS(visitor.at(3));
  }

  // Test Visitor
  SUBCASE("Visitor") {
    ConstraintMap map;
    map.emplace_back(1, 2, 3);
    map.emplace_back(4, 5, 6);

    ConstraintMap::Visitor visitor = *(map.begin());

    CHECK(visitor.size() == 3);
    CHECK(visitor[0] == 1);
    CHECK(visitor[1] == 2);
    CHECK(visitor[2] == 3);
    CHECK_THROWS(visitor.at(3));

    visitor[0] = 7;
    CHECK(visitor[0] == 7);
  }

  // Test Iterator
  SUBCASE("Iterator") {
    ConstraintMap map;
    map.emplace_back(1, 2, 3);
    map.emplace_back(4, 5, 6);

    ConstraintMap::Iterator it = map.begin();

    CHECK(it != map.end());
    CHECK((*it).size() == 3);
    CHECK((*it)[0] == 1);
    CHECK((*it)[1] == 2);
    CHECK((*it)[2] == 3);

    ++it;
    CHECK(it != map.end());
    CHECK((*it).size() == 3);
    CHECK((*it)[0] == 4);
    CHECK((*it)[1] == 5);
    CHECK((*it)[2] == 6);

    ++it;
    CHECK(it == map.end());
  }
}