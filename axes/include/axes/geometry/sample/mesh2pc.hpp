#pragma once

#include "axes/geometry/common.hpp"
#include "axes/utils/status.hpp"

namespace ax::geo {

/************************* SECT: Mesh to Point Cloud *************************/

template <idx dim> class MeshPointCloudSampler {
public:
  MeshPointCloudSampler(SurfaceMesh const& mesh, math::fieldr<dim> const& interpolations)
      : vertices_(mesh.first), indices_(mesh.second), interpolations_(interpolations) {}

  template <idx out_dim>
  StatusOr<math::fieldr<out_dim>> Sample(math::fieldr<out_dim> const& input) const {
    idx n_interp = interpolations_.cols();
    idx n_points = vertices_.cols();
    idx n_input = input.cols();
    if (n_input != n_points) {
      return utils::InvalidArgumentError(
          "Input must have the same number of columns as the vertices.");
    }

    math::fieldr<out_dim> result;
    result.resize(out_dim, n_interp * n_points);
    for (idx i = 0; i < n_points; ++i) {
      for (idx j = 0; j < n_interp; ++j) {
        math::vecr<out_dim> local = math::zeros<out_dim>();
        for (idx k = 0; k < dim; ++k) {
          local += interpolations_(k, j) * input.col(indices_(k, i));
        }
        result.col(i * n_interp + j) = local;
      }
    }

    return result;
  }

private:
  math::fieldr<dim> const vertices_;
  math::fieldi<dim> const indices_;
  math::fieldr<dim> const interpolations_;
};

}  // namespace ax::geo
