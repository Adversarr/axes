#include <doctest/doctest.h>

#include <entt/core/hashed_string.hpp>
#include <entt/locator/locator.hpp>

#include "ax/math/lattice.hpp"

TEST_CASE("Lattice2D") {
  ax::math::Lattice<2, float> lattice({2, 3});
  CHECK(lattice.Shape() == ax::math::IndexVector<2>{2, 3});
  CHECK(lattice.Stride() == ax::math::IndexVector<2>{3, 1});
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
  ax::math::Lattice<2, float> lattice({2, 2}, ax::math::staggered);
  CHECK(lattice.Shape() == ax::math::IndexVector<2>{2, 2});
  CHECK(lattice.Stride() == ax::math::IndexVector<2>{3, 1});

  lattice(0, 0) = 1;
  lattice(1, 0) = 2;
  lattice(2, 0) = 3;
  lattice(0, 1) = 4;
  lattice(1, 1) = 5;
  lattice(2, 1) = 6;
}