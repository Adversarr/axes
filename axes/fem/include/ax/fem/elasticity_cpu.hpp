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
template <Index dim, template <Index> class ElasticModelTemplate> class ElasticityCompute_CPU
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using elem_stress_t = elasticity::vector_for_eigen_type<math::RealMatrix<dim, dim>>;
  using vert_stress_t = math::RealField<dim>;
  using elem_hessian_t = elasticity::vector_for_eigen_type<math::RealMatrix<dim * dim, dim * dim>>;
  using vert_hessian_t = math::spmatr;
  using MeshPtr = std::shared_ptr<TriMesh<dim>>;
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  explicit ElasticityCompute_CPU(std::shared_ptr<TriMesh<dim>> mesh);
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
  std::vector<math::decomp::SvdResult<dim, real>> svd_results_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
  struct TbbPartitioners;  ///< include tbb in CUDA will cause error, so we have to put it here.
  std::unique_ptr<TbbPartitioners> partitioner_impl_;
};
}  // namespace ax::fem
