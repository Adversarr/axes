#pragma once
#include "elasticity.hpp"

namespace ax::fem {

template <int dim, template <int> class ElasticModelTemplate> class ElasticityCompute_GPU
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using elem_stress_t = math::aligned_vector<math::RealMatrix<dim, dim>>;
  using vert_stress_t = math::RealField<dim>;
  using elem_hessian_t = math::aligned_vector<math::RealMatrix<dim * dim, dim * dim>>;
  using vert_hessian_t = math::RealSparseMatrix;
  using MeshPtr = std::shared_ptr<TriMesh<dim>>;
  explicit ElasticityCompute_GPU(MeshPtr const& mesh);

  ~ElasticityCompute_GPU() override;

  void Update(math::RealField<dim> const& pose, ElasticityUpdateLevel update_type) final;
  void RecomputeRestPose() final;
  void SetLame(math::RealVector2 const& u_lame) final;
  void SetLame(math::RealField2 const& e_lame) final;

  void UpdateEnergy() final;
  void UpdateStress() final;
  void UpdateHessian(bool projection) final;

  void GatherEnergyToVertices() final;
  void GatherStressToVertices() final;
  void GatherHessianToVertices() final;

  elem_hessian_t const& GetHessianOnElements() final;
  vert_hessian_t const& GetHessianOnVertices() final;
  elem_stress_t const& GetStressOnElements() final;
  vert_stress_t const& GetStressOnVertices() final;
  math::RealField1 const& GetEnergyOnElements() final;
  math::RealField1 const& GetEnergyOnVertices() final;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  // NOTE: Currently, GPU does not support SVD decomposition.
};

}  // namespace ax::fem
