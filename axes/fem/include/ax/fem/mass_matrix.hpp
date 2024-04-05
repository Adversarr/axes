#pragma once
#include "mesh_base.hpp"
#include "ax/math/sparse.hpp"

namespace ax::fem {

/**
 * @brief Compute the mass matrix for the given mesh.
 * 
 * @tparam dim 
 */
template <idx dim> class MassMatrixCompute {
public:
  MassMatrixCompute(MeshBase<dim> const& mesh) : mesh_(&mesh) {}
  /**
   * @brief Compute the mass matrix.
   * 
   * @param density density field.
   * @param is_density_on_element if true, the density is defined on elements, otherwise on vertices.
   * @return math::sp_coeff_list 
   */
  math::sp_coeff_list operator()(math::field1r const& density, bool is_density_on_element);

  /**
   * @brief Compute the mass matrix.
   * 
   * @param density Uniform density value.
   * @return math::sp_coeff_list 
   */
  math::sp_coeff_list operator()(real density);

private:
  MeshBase<dim> const* mesh_;
  bool compute_lamped_;
};

}  // namespace ax::fem