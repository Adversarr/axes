#pragma once
#include "elasticity.hpp"

namespace ax::fem {

template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute_GPU
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using elem_stress_t = List<math::matr<dim, dim>>;
  using vert_stress_t = math::fieldr<dim>;
  using elem_hessian_t = List<math::matr<dim * dim, dim * dim>>;
  using vert_hessian_t = math::spmatr;
  using MeshPtr = SPtr<TriMesh<dim>>;
  ElasticityCompute_GPU(MeshPtr const& mesh);

  ~ElasticityCompute_GPU();

  bool Update(math::fieldr<dim> const& pose, ElasticityUpdateLevel update_type) final;
  void RecomputeRestPose() final;
  void SetLame(math::vec2r const& u_lame) final;
  void SetLame(math::field2r const& e_lame) final;

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
  math::field1r const& GetEnergyOnElements() final;
  math::field1r const& GetEnergyOnVertices() final;

private:
  struct Impl;
  UPtr<Impl> impl_;
  // NOTE: Currently, GPU does not support SVD decomposition.
};

}  // namespace ax::fem
