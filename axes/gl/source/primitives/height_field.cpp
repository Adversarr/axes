#include "ax/gl/primitives/height_field.hpp"

#include "ax/core/logging.hpp"
#include "ax/geometry/grid.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/primitives/mesh.hpp"

namespace ax::gl {

Mesh make_height_field(math::vecxr const& z, idx nx, idx ny) {
  Mesh mesh;
  AX_CHECK(z.size() == nx * ny, "Size mismatch");
  mesh.vertices_.resize(3, nx * ny);
  for (idx i = 0; i < nx; ++i) {
    for (idx j = 0; j < ny; ++j) {
      real x = static_cast<real>(i) / static_cast<real>(nx - 1);
      real y = static_cast<real>(j) / static_cast<real>(ny - 1);
      real zi = z[i * ny + j];
      mesh.vertices_.col(i * ny + j) = math::vec3r(x, y, zi);
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
