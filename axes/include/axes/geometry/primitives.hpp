#pragma once

#include "axes/geometry/common.hpp"
#include "axes/math/common.hpp"
namespace ax::geo {

SurfaceMesh cube(real half_size);

SurfaceMesh sphere(real radius, idx slices, idx stacks);


}  // namespace ax::geo
