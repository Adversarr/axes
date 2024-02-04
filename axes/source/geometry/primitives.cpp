#include "axes/geometry/primitives.hpp"

namespace ax::geo {

std::pair<math::field3r, math::field3i> cube(real size) {
  math::field3r vertices(3, 8);
  vertices.col(0) = math::vec3r{-size, -size, -size};
  vertices.col(1) = math::vec3r{size, -size, -size};
  vertices.col(2) = math::vec3r{size, size, -size};
  vertices.col(3) = math::vec3r{-size, size, -size};
  vertices.col(4) = math::vec3r{-size, -size, size};
  vertices.col(5) = math::vec3r{size, -size, size};
  vertices.col(6) = math::vec3r{size, size, size};
  vertices.col(7) = math::vec3r{-size, size, size};

  math::field3i indices(3, 12);
  indices.col(0) = math::vec3i{0, 1, 2};
  indices.col(1) = math::vec3i{2, 3, 0};
  indices.col(2) = math::vec3i{1, 5, 6};
  indices.col(3) = math::vec3i{6, 2, 1};
  indices.col(4) = math::vec3i{7, 6, 5};
  indices.col(5) = math::vec3i{5, 4, 7};
  indices.col(6) = math::vec3i{4, 0, 3};
  indices.col(7) = math::vec3i{3, 7, 4};
  indices.col(8) = math::vec3i{4, 5, 1};
  indices.col(9) = math::vec3i{1, 0, 4};
  indices.col(10) = math::vec3i{3, 2, 6};
  indices.col(11) = math::vec3i{6, 7, 3};

  return {vertices, indices};
}

}
