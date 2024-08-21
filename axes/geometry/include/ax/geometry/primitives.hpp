#pragma once

#include "ax/geometry/common.hpp"
#include "ax/math/common.hpp"
namespace ax::geo {

SurfaceMesh cube(Real half_size);

SurfaceMesh sphere(Real radius, Index slices, Index stacks);

SurfaceMesh plane(Real half_width, Real half_height, Index nx, Index ny);

TetraMesh tet_cube(Real half_size, Index nx, Index ny, Index nz);

}  // namespace ax::geo
