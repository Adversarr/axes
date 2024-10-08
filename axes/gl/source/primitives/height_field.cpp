#include "ax/gl/primitives/height_field.hpp"

#include "ax/core/logging.hpp"
#include "ax/geometry/grid.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/primitives/mesh.hpp"

namespace ax::gl {

Mesh make_height_field(math::RealVectorX const& z, Index nx, Index ny) {
  Mesh mesh;
  AX_CHECK(z.size() == nx * ny, "Size mismatch");
  mesh.vertices_.resize(3, nx * ny);
  for (Index i = 0; i < nx; ++i) {
    for (Index j = 0; j < ny; ++j) {
      Real x = static_cast<Real>(i) / static_cast<Real>(nx - 1);
      Real y = static_cast<Real>(j) / static_cast<Real>(ny - 1);
      Real zi = z[i * ny + j];
      mesh.vertices_.col(i * ny + j) = math::RealVector3(x, y, zi);
    }
  }

  mesh.use_lighting_ = false;
  mesh.is_flat_ = false;
  mesh.indices_ = geo::make_grid_triangles(nx, ny);
  mesh.colors_.resize(4, nx * ny);
  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  Colormap colormap(z.minCoeff(), z.maxCoeff(), false, colormap_coolwarm);
  mesh.colors_.topRows<3>() = colormap(z);
  mesh.colors_.row(3).setOnes();
  return mesh;
}

}  // namespace ax::gl
