#pragma once
#include "ax/math/decomp/svd/common.hpp"
#include "elasticity.hpp"

namespace ax::fem {

/**
 * @brief Implementation of the elasticity computation.
 *
 * @tparam dim
 * @tparam ElasticModelTemplate
 */
template <int dim, template <int> class ElasticModelTemplate> class ElasticityCompute_CPU
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using elem_stress_t = math::aligned_vector<math::RealMatrix<dim, dim>>;
  using vert_stress_t = math::RealField<dim>;
  using elem_hessian_t = math::aligned_vector<math::RealMatrix<dim * dim, dim * dim>>;
  using vert_hessian_t = math::RealSparseMatrix;
  using MeshPtr = std::shared_ptr<LinearMesh<dim>>;
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  explicit ElasticityCompute_CPU(std::shared_ptr<LinearMesh<dim>> mesh);
  ~ElasticityCompute_CPU() override;

  void UpdateEnergy() final;
  void UpdateStress() final;
  void UpdateHessian(bool projection) final;

  void GatherEnergyToVertices() final;
  void GatherStressToVertices() final;
  void GatherHessianToVertices() final;
  void Update(math::RealField<dim> const& pose, ElasticityUpdateLevel update_type) final;
  void RecomputeRestPose() final;

protected:
  std::vector<math::decomp::SvdResult<dim, Real>> svd_results_;
  elasticity::DeformGradBuffer<dim> deformation_gradient_;
  struct Impl;  ///< include tbb in CUDA will cause error, so we have to put it here.
  std::unique_ptr<Impl> impl_;
};
}  // namespace ax::fem
