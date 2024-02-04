#include "axes/geometry/primitives.hpp"

namespace ax::geo {

std::pair<math::field3r, math::field3i> cube(real size) {
  /**
    var DEFAULT_VERT = [
        1, 1, 1, 1, 1,1,1,1,
        -1, 1, 1, 1, 1,0,0,1,
        -1,-1, 1, 1, 0,1,0,1,
        1,-1, 1, 1, 0,0,1,1,
        1,-1,-1, 1, 0,1,1,1,
        1, 1,-1, 1, 1,1,0,1,
        -1, 1,-1, 1, 1,0,1,1,
        -1,-1,-1, 1, 0,0,0,1,
      ];

    var DEFAULT_INDICES = new Uint8Array([
        0, 1, 2,   0, 2, 3,    // front
        0, 3, 4,   0, 4, 5,    // right
        0, 5, 6,   0, 6, 1,    // up
        1, 6, 7,   1, 7, 2,    // left
        7, 4, 3,   7, 3, 2,    // down
        4, 7, 6,   4, 6, 5     // back
    ]);
   */
  math::field3r vertices(3, 8);
  math::field3i indices(3, 12);

  vertices.col(0) = math::vec3r{1, 1, 1};
  vertices.col(1) = math::vec3r{-1, 1, 1};
  vertices.col(2) = math::vec3r{-1, -1, 1};
  vertices.col(3) = math::vec3r{1, -1, 1};
  vertices.col(4) = math::vec3r{1, -1, -1};
  vertices.col(5) = math::vec3r{1, 1, -1};
  vertices.col(6) = math::vec3r{-1, 1, -1};
  vertices.col(7) = math::vec3r{-1, -1, -1};

  indices.col(0) = math::vec3i{0, 1, 2};
  indices.col(1) = math::vec3i{0, 2, 3};
  indices.col(2) = math::vec3i{0, 3, 4};
  indices.col(3) = math::vec3i{0, 4, 5};
  indices.col(4) = math::vec3i{0, 5, 6};
  indices.col(5) = math::vec3i{0, 6, 1};
  indices.col(6) = math::vec3i{1, 6, 7};
  indices.col(7) = math::vec3i{1, 7, 2};
  indices.col(8) = math::vec3i{7, 4, 3};
  indices.col(9) = math::vec3i{7, 3, 2};
  indices.col(10) = math::vec3i{4, 7, 6};
  indices.col(11) = math::vec3i{4, 6, 5};

  vertices *= size;

  return {vertices, indices};
}

}  // namespace ax::geo
