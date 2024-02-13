#pragma once

#include "axes/math/common.hpp"

namespace ax::geo {

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

}