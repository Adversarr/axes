#pragma once

#include "axes/geometry/common.hpp"
#include "axes/utils/status.hpp"

namespace ax::geo {

/************************* SECT: Mesh to Point Cloud *************************/

/**
 * @brief A class that samples a mesh to generate a point cloud.
 *
 * This class takes a surface mesh and a set of interpolations as input, and provides a method to sample
 * the mesh based on an input field. The resulting sampled points are stored in a field.
 *
 * @tparam dim The dimension of the mesh.
 */
template <idx dim> class MeshPointCloudSampler {
public:
  /**
   * @brief Constructs a MeshPointCloudSampler object.
   *
   * @param mesh The surface mesh to sample.
   * @param interpolations The interpolations to use for sampling.
   */
  MeshPointCloudSampler(SurfaceMesh const& mesh, math::fieldr<dim> const& interpolations)
      : vertices_(mesh.first), indices_(mesh.second), interpolations_(interpolations) {}

  /**
   * @brief Samples the mesh based on the input field.
   *
   * This method takes an input field and samples the mesh based on it. The sampled points are stored
   * in a field of the specified output dimension.
   *
   * @tparam out_dim The dimension of the output field.
   * @param input The input field to use for sampling.
   * @return A StatusOr object containing the sampled points as a field of the specified output dimension.
   *         If an error occurs during sampling, an error status is returned.
   */
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
  math::fieldr<dim> const vertices_;        ///< The vertices of the mesh.
  math::fieldi<dim> const indices_;         ///< The indices of the mesh.
  math::fieldr<dim> const interpolations_;  ///< The interpolations to use for sampling.
};

}  // namespace ax::geo
