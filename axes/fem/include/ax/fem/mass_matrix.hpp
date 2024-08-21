#pragma once
#include "ax/math/sparse.hpp"
#include "trimesh.hpp"

namespace ax::fem {

/**
 * @brief Compute the mass matrix for the given mesh.
 *
 * @tparam dim
 */
template <int dim> class MassMatrixCompute {
public:
  explicit MassMatrixCompute(TriMesh<dim> const& mesh) : mesh_(mesh) {}
  /**
   * @brief Compute the mass matrix.
   *
   * @param density density field.
   * @param is_density_on_element if true, the density is defined on elements, otherwise on
   * vertices.
   * @return math::sp_coeff_list
   */
  math::RealSparseMatrix operator()(math::RealField1 const& density);

  /**
   * @brief Compute the mass matrix.
   *
   * @param density Uniform density value.
   * @return math::sp_coeff_list
   */
  math::RealSparseMatrix operator()(Real density);

  math::RealVectorX Lumped(math::RealField1 const& density, bool is_density_on_element);

  math::RealVectorX Lumped(Real density);

private:
  TriMesh<dim> const& mesh_;
  bool compute_lamped_;
};

}  // namespace ax::fem
