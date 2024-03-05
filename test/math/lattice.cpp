#include <doctest/doctest.h>

#include <entt/core/hashed_string.hpp>
#include <entt/locator/locator.hpp>

#include "axes/math/lattice.hpp"

TEST_CASE("Lattice2D") {
  ax::math::Lattice<2, float> lattice(2, 3);
  CHECK(lattice.Shape() == ax::math::veci<2>{2, 3});
  CHECK(lattice.Strides() == ax::math::veci<2>{3, 1});
  lattice(0, 0) = 1;
  lattice(0, 1) = 2;
  lattice(0, 2) = 3;
  lattice(1, 0) = 4;
  lattice(1, 1) = 5;
  lattice(1, 2) = 6;
  CHECK(lattice(0, 0) == 1);
  CHECK(lattice(0, 1) == 2);
  CHECK(lattice(0, 2) == 3);
  CHECK(lattice(1, 0) == 4);
  CHECK(lattice(1, 1) == 5);
  CHECK(lattice(1, 2) == 6);
}


/**
 * @brief StaggeredLattice represents a grid which stores the value at the faces of each cell.
 *
 *  ^ y
 *  |
 *  +--yt--+
 *  |      |
 *  xl     xr
 *  |      |   x
 *  +--yb--+-- >
 *
 * The subscript is (k, i, j) where k is the direction, i is the x-axis, and j is the y-axis. e.g.
 *   - xl subscript is (0, 0, 0), xr is (0, 1, 0), yt is (1, 0, 0), and yb is (1, 0, 1).
 * 
 * Possible Usage is a velocity field in fluid simulation.
 *
 */
TEST_CASE("StaggeredLattice2D") {
  ax::math::StaggeredLattice<2, float> lattice({2, 2});
  CHECK(lattice.Shape() == ax::math::veci<2>{2, 2});
  CHECK(lattice.X().Shape() == ax::math::veci<2>{3, 2});
  CHECK(lattice.Y().Shape() == ax::math::veci<2>{2, 3});

  lattice(0, 0, 0) = 1;
  lattice(0, 1, 0) = 2;
  lattice(0, 2, 0) = 3;
  lattice(0, 0, 1) = 4;
  lattice(0, 1, 1) = 5;
  lattice(0, 2, 1) = 6;

  lattice(1, 0, 0) = 7;
  lattice(1, 0, 1) = 8;
  lattice(1, 0, 2) = 9;
  lattice(1, 1, 0) = 10;
  lattice(1, 1, 1) = 11;
  lattice(1, 1, 2) = 12;

  CHECK(lattice(0, 0, 0) == 1);
  CHECK(lattice(0, 1, 0) == 2);
  CHECK(lattice(0, 2, 0) == 3);
  CHECK(lattice(0, 0, 1) == 4);
  CHECK(lattice(0, 1, 1) == 5);
  CHECK(lattice(0, 2, 1) == 6);
  CHECK(lattice(1, 0, 0) == 7);
  CHECK(lattice(1, 0, 1) == 8);
  CHECK(lattice(1, 0, 2) == 9);
  CHECK(lattice(1, 1, 0) == 10);
  CHECK(lattice(1, 1, 1) == 11);
  CHECK(lattice(1, 1, 2) == 12);

  auto div = lattice.Divergence();
  for (auto [i, j]: ax::utils::multi_iota(2, 2)) {
    CHECK(div(i, j) == 2);
  }

  auto cell_centered = lattice.ToCellCentered();
  CHECK(cell_centered(0, 0) == ax::math::vec<float, 2>{1.5, 7.5});
}