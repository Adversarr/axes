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
inline math::RealField2 meshgrid(math::RealVectorX const& x, math::RealVectorX const& y) {
  Index nx = math::rows(x);
  Index ny = math::rows(y);
  math::RealField2 X(2, nx * ny);
  for (Index i = 0; i < nx; ++i) {
    for (Index j = 0; j < ny; ++j) {
      X.col(i * ny + j) = math::RealVector2{x(i), y(j)};
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
inline math::IndexField3 make_grid_triangles(Index nx, Index ny) {
  math::IndexField3 triangles(3, 2 * (nx - 1) * (ny - 1));
  Index id = 0;
  for (Index i = 0; i < nx - 1; ++i) {
    for (Index j = 0; j < ny - 1; ++j) {
      Index Index00 = i * ny + j;
      Index Index01 = i * ny + j + 1;
      Index Index10 = (i + 1) * ny + j;
      Index Index11 = (i + 1) * ny + j + 1;
      triangles.col(id++) = math::IndexVec3{Index00, Index11, Index01};
      triangles.col(id++) = math::IndexVec3{Index00, Index11, Index10};
    }
  }
  return triangles;
}

}  // namespace ax::geo
