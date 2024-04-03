#pragma once

#include "ax/math/common.hpp"
namespace ax::geo {

/**
 * @brief Generates a meshgrid of points given x and y vectors.
 *
 * This function generates a meshgrid of points by combining the elements of the x and y vectors.
 * The resulting meshgrid is represented as a 2D field of points.
 *
 * @param x The x vector.
 * @param y The y vector.
 * @return A 2D field of points representing the meshgrid.
 */
inline math::field2r meshgrid(math::vecxr const& x, math::vecxr const& y) {
  idx nx = math::rows(x);
  idx ny = math::rows(y);
  math::field2r X(2, nx * ny);
  for (idx i = 0; i < nx; ++i) {
    for (idx j = 0; j < ny; ++j) {
      X.col(i * ny + j) = math::vec2r{x(i), y(j)};
    }
  }
  return X;
}

/**
 * @brief Generates a grid of triangles for a given number of points in the x and y directions.
 *
 * This function generates a grid of triangles for a given number of points in the x and y
 * directions. The resulting grid is represented as a 2D field of triangles.
 *
 * @param nx The number of points in the x direction.
 * @param ny The number of points in the y direction.
 * @return A 2D field of triangles representing the grid.
 */
inline math::field3i make_grid_triangles(idx nx, idx ny) {
  math::field3i triangles(3, 2 * (nx - 1) * (ny - 1));
  idx id = 0;
  for (idx i = 0; i < nx - 1; ++i) {
    for (idx j = 0; j < ny - 1; ++j) {
      idx idx00 = i * ny + j;
      idx idx01 = i * ny + j + 1;
      idx idx10 = (i + 1) * ny + j;
      idx idx11 = (i + 1) * ny + j + 1;
      triangles.col(id++) = math::vec3i{idx00, idx11, idx01};
      triangles.col(id++) = math::vec3i{idx00, idx11, idx10};
    }
  }
  return triangles;
}

}  // namespace ax::geo
