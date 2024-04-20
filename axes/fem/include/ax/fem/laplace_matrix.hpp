#pragma once
#include "ax/fem/mesh_base.hpp"
#include "ax/math/sparse.hpp"

namespace ax::fem {

/**
 * @brief Compute the mass matrix for the given mesh.
 * 
 * @tparam dim 
 */
template <idx dim> class LaplaceMatrixCompute {
public:
  explicit LaplaceMatrixCompute(MeshBase<dim> const& mesh) : mesh_(&mesh) {}

  /**
   * @brief Compute the Laplace matrix.
   * 
   * @param W weight for the Laplace operator.
   * @return math::sp_coeff_list 
   */
  math::sp_matxxr operator()(real W = 1.0);

  /**
   * @brief Compute the Laplace matrix.
   * 
   * @param W weight for the Laplace operator, per element.
   * @return math::sp_coeff_list 
   */
  math::sp_matxxr operator()(math::field1r const& W);

private:
  MeshBase<dim> const* mesh_;
  bool compute_lamped_;
};

}