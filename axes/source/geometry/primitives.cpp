#include "axes/geometry/primitives.hpp"
#include "axes/geometry/grid.hpp"

namespace ax::geo {

SurfaceMesh cube(real size) {
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

SurfaceMesh sphere(real radius, idx slices, idx stacks) {
  math::field3r vertices(3, (slices + 1) * (stacks + 1));
  math::field3i indices(3, 2 * slices * stacks);

  for (idx i = 0; i <= stacks; ++i) {
    real const theta = math::pi<real> * i / stacks;
    real const sin_theta = math::sin(theta);
    real const cos_theta = math::cos(theta);

    for (idx j = 0; j <= slices; ++j) {
      real const phi = 2 * math::pi<real> * j / slices;
      real const sin_phi = math::sin(phi);
      real const cos_phi = math::cos(phi);

      vertices.col(i * (slices + 1) + j)
          = math::vec3r{radius * cos_phi * sin_theta, radius * sin_phi * sin_theta, radius * cos_theta};
    }
  }

  idx k = 0;
  for (idx i = 0; i < stacks; ++i) {
    for (idx j = 0; j < slices; ++j) {
      indices.col(k++) = math::vec3i{i * (slices + 1) + j, (i + 1) * (slices + 1) + j, (i + 1) * (slices + 1) + j + 1};
      indices.col(k++) = math::vec3i{i * (slices + 1) + j, (i + 1) * (slices + 1) + j + 1, i * (slices + 1) + j + 1};
    }
  }

  return {vertices, indices};
}

SurfaceMesh plane(real half_width, real half_height, idx nx, idx ny){
  math::field3r V(3, (nx + 1) * (ny + 1)); // 3D field of points (vertices
  math::field3i F;
  auto X = math::linspace(-half_width, half_width, nx + 1);
  auto Y = math::linspace(-half_height, half_height, ny + 1);
  V.topRows<2>() = geo::meshgrid(X, Y);
  V.row(2).setZero();
  F = geo::make_grid_triangles(nx + 1, ny + 1);
  return {V, F};
}

}  // namespace ax::geo
