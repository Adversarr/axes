#pragma once

#include "ax/core/excepts.hpp"
#include "ax/geometry/common.hpp"

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
template <int dim> class MeshPointCloudSampler {
public:
  /**
   * @brief Constructs a MeshPointCloudSampler object.
   *
   * @param mesh The surface mesh to sample.
   * @param interpolations The interpolations to use for sampling.
   */
  MeshPointCloudSampler(SurfaceMesh const& mesh, math::RealField<dim> const& interpolations)
      : vertices_(mesh.vertices_), indices_(mesh.indices_), interpolations_(interpolations) {}

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
  template <Index out_dim>
  math::RealField<out_dim> Sample(math::RealField<out_dim> const& input) const {
    Index n_interp = interpolations_.cols();
    Index n_points = vertices_.cols();
    Index n_input = input.cols();
    if (n_input != n_points) {
      throw make_invalid_argument("Input must have the same number of columns as the vertices.");
    }

    math::RealField<out_dim> result;
    result.resize(out_dim, n_interp * n_points);
    for (Index i = 0; i < n_points; ++i) {
      for (Index j = 0; j < n_interp; ++j) {
        math::RealVector<out_dim> local = math::zeros<out_dim>();
        for (Index k = 0; k < dim; ++k) {
          local += interpolations_(k, j) * input.col(indices_(k, i));
        }
        result.col(i * n_interp + j) = local;
      }
    }

    return result;
  }

private:
  math::RealField<dim> const vertices_;        ///< The vertices of the mesh.
  math::IndexField<dim> const indices_;         ///< The indices of the mesh.
  math::RealField<dim> const interpolations_;  ///< The interpolations to use for sampling.
};

}  // namespace ax::geo
