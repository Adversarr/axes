#pragma once

#include "axes/geometry/common.hpp"
#include "axes/math/common.hpp"
namespace ax::geo {

SurfaceMesh cube(real half_size);

SurfaceMesh sphere(real radius, idx slices, idx stacks);

SurfaceMesh plane(real half_width, real half_height, idx nx, idx ny);

TetraMesh tet_cube(real half_size, idx nx, idx ny, idx nz);

}  // namespace ax::geo
