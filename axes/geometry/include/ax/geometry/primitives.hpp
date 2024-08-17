#pragma once

#include "ax/geometry/common.hpp"
#include "ax/math/common.hpp"
namespace ax::geo {

SurfaceMesh cube(real half_size);

SurfaceMesh sphere(real radius, Index slices, Index stacks);

SurfaceMesh plane(real half_width, real half_height, Index nx, Index ny);

TetraMesh tet_cube(real half_size, Index nx, Index ny, Index nz);

}  // namespace ax::geo
