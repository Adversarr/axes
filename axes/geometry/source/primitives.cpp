#include "ax/geometry/primitives.hpp"
#include "ax/geometry/grid.hpp"

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
  math::RealField3 vertices(3, 8);
  math::IndexField3 indices(3, 12);

  vertices.col(0) = math::RealVector3{1, 1, 1};
  vertices.col(1) = math::RealVector3{-1, 1, 1};
  vertices.col(2) = math::RealVector3{-1, -1, 1};
  vertices.col(3) = math::RealVector3{1, -1, 1};
  vertices.col(4) = math::RealVector3{1, -1, -1};
  vertices.col(5) = math::RealVector3{1, 1, -1};
  vertices.col(6) = math::RealVector3{-1, 1, -1};
  vertices.col(7) = math::RealVector3{-1, -1, -1};

  indices.col(0) = math::IndexVec3{0, 1, 2};
  indices.col(1) = math::IndexVec3{0, 2, 3};
  indices.col(2) = math::IndexVec3{0, 3, 4};
  indices.col(3) = math::IndexVec3{0, 4, 5};
  indices.col(4) = math::IndexVec3{0, 5, 6};
  indices.col(5) = math::IndexVec3{0, 6, 1};
  indices.col(6) = math::IndexVec3{1, 6, 7};
  indices.col(7) = math::IndexVec3{1, 7, 2};
  indices.col(8) = math::IndexVec3{7, 4, 3};
  indices.col(9) = math::IndexVec3{7, 3, 2};
  indices.col(10) = math::IndexVec3{4, 7, 6};
  indices.col(11) = math::IndexVec3{4, 6, 5};

  vertices *= size;

  return {vertices, indices};
}

SurfaceMesh sphere(real radius, Index slices, Index stacks) {
  math::RealField3 vertices(3, (slices + 1) * (stacks + 1));
  math::IndexField3 indices(3, 2 * slices * stacks);

  for (Index i = 0; i <= stacks; ++i) {
    real const theta = math::pi<real> * i / stacks;
    real const sin_theta = math::sin(theta);
    real const cos_theta = math::cos(theta);

    for (Index j = 0; j <= slices; ++j) {
      real const phi = 2 * math::pi<real> * j / slices;
      real const sin_phi = math::sin(phi);
      real const cos_phi = math::cos(phi);

      vertices.col(i * (slices + 1) + j)
          = math::RealVector3{radius * cos_phi * sin_theta, radius * sin_phi * sin_theta, radius * cos_theta};
    }
  }

  Index k = 0;
  for (Index i = 0; i < stacks; ++i) {
    for (Index j = 0; j < slices; ++j) {
      indices.col(k++) = math::IndexVec3{i * (slices + 1) + j, (i + 1) * (slices + 1) + j, (i + 1) * (slices + 1) + j + 1};
      indices.col(k++) = math::IndexVec3{i * (slices + 1) + j, (i + 1) * (slices + 1) + j + 1, i * (slices + 1) + j + 1};
    }
  }

  return {vertices, indices};
}

SurfaceMesh plane(real half_width, real half_height, Index nx, Index ny){
  math::RealField3 V(3, (nx + 1) * (ny + 1)); // 3D field of points (vertices
  math::IndexField3 F;
  auto X = math::linspace(-half_width, half_width, nx + 1);
  auto Y = math::linspace(-half_height, half_height, ny + 1);
  V.topRows<2>() = geo::meshgrid(X, Y);
  V.row(2).setZero();
  F = geo::make_grid_triangles(nx + 1, ny + 1);
  return {V, F};
}

TetraMesh tet_cube(real half_size, Index nx, Index ny, Index nz) {
  using namespace ax::math;
  RealField3 vertices(3, nx * ny * nz);
  IndexField4 elements(4, 5 * (nx - 1) * (ny - 1) * (nz - 1));
  Index id = 0;
  for (Index i = 0; i < nx; ++i) {
    for (Index j = 0; j < ny; ++j) {
      for (Index k = 0; k < nz; ++k) {
        vertices.col(id) = RealVector3{i / real(nx - 1), j / real(ny - 1), k / real(nz - 1)};
        id++;
      }
    }
  }
  vertices.array() -= 0.5;
  vertices.array() *= 2 * half_size;

  id = 0;
  for (Index i = 0; i < nx - 1; ++i) {
    for (Index j = 0; j < ny - 1; ++j) {
      for (Index k = 0; k < nz - 1; ++k) {
        Index Index000 = i * ny * nz + j * nz + k;
        Index Index100 = (i + 1) * ny * nz + j * nz + k;
        Index Index010 = i * ny * nz + (j + 1) * nz + k;
        Index Index001 = i * ny * nz + j * nz + k + 1;
        Index Index110 = (i + 1) * ny * nz + (j + 1) * nz + k;
        Index Index101 = (i + 1) * ny * nz + j * nz + k + 1;
        Index Index011 = i * ny * nz + (j + 1) * nz + k + 1;
        Index Index111 = (i + 1) * ny * nz + (j + 1) * nz + k + 1;
        if (i % 2 == 1) {
          // Flip in x direction:
          std::swap(Index000, Index100);
          std::swap(Index010, Index110);
          std::swap(Index001, Index101);
          std::swap(Index011, Index111);
        }

        if (j % 2 == 1) {
          // Flip in y direction:
          std::swap(Index000, Index010);
          std::swap(Index100, Index110);
          std::swap(Index001, Index011);
          std::swap(Index101, Index111);
        }

        if (k % 2 == 1) {
          // Flip in z direction:
          std::swap(Index000, Index001);
          std::swap(Index100, Index101);
          std::swap(Index010, Index011);
          std::swap(Index110, Index111);
        }

        elements.col(id++) = IndexVec4{Index000, Index100, Index010, Index001};
        elements.col(id++) = IndexVec4{Index010, Index100, Index110, Index111};
        elements.col(id++) = IndexVec4{Index100, Index010, Index001, Index111};
        elements.col(id++) = IndexVec4{Index100, Index001, Index101, Index111};
        elements.col(id++) = IndexVec4{Index011, Index001, Index010, Index111};
      }
    }
  }

  return {vertices, elements};
}
}  // namespace ax::geo
