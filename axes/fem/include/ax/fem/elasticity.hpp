#pragma once
#include "ax/math/decomp/svd/common.hpp"
#include "ax/math/sparse.hpp"
#include "elasticity/common.hpp"
#include "trimesh.hpp"

namespace ax::fem {

AX_DECLARE_ENUM(ElasticityUpdateLevel){kEnergy, kStress, kHessian};

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  using elem_stress_t = List<math::matr<dim, dim>>;
  using vert_stress_t = math::fieldr<dim>;
  using elem_hessian_t = List<math::matr<dim * dim, dim * dim>>;
  using vert_hessian_t = math::spmatr;
  using MeshPtr = SPtr<TriMesh<dim>>;

  explicit ElasticityComputeBase(SPtr<TriMesh<dim>> mesh);
  virtual ~ElasticityComputeBase() = default;

  void SetMesh(MeshPtr const& mesh);
  virtual void RecomputeRestPose();

  virtual void SetLame(math::vec2r const& u_lame);
  virtual void SetLame(math::field2r const& e_lame);

  virtual bool Update(math::fieldr<dim> const& pose, ElasticityUpdateLevel update_type) = 0;

  virtual void UpdateEnergy() = 0;
  virtual void UpdateStress() = 0;
  virtual void UpdateHessian(bool projection) = 0;

  virtual void GatherEnergyToVertices() = 0;
  virtual void GatherStressToVertices() = 0;
  virtual void GatherHessianToVertices() = 0;

  virtual elem_hessian_t const& GetHessianOnElements() { return hessian_on_elements_; }
  virtual vert_hessian_t const& GetHessianOnVertices() { return hessian_on_vertices_; }
  virtual elem_stress_t const& GetStressOnElements() { return stress_on_elements_; }
  virtual vert_stress_t const& GetStressOnVertices() { return stress_on_vertices_; }
  virtual math::field1r const& GetEnergyOnElements() { return energy_on_elements_; }
  virtual math::field1r const& GetEnergyOnVertices() { return energy_on_vertices_; }

protected:
  SPtr<TriMesh<dim>> mesh_;
  elasticity::DeformationGradientCache<dim> rinv_;
  math::field1r rest_volume_;
  math::field2r lame_;

  math::field1r energy_on_elements_;
  math::field1r energy_on_vertices_;
  elem_stress_t stress_on_elements_;
  vert_stress_t stress_on_vertices_;
  elem_hessian_t hessian_on_elements_;
  vert_hessian_t hessian_on_vertices_;
};

/**
 * @brief Implementation of the elasticity computation.
 *
 * @tparam dim
 * @tparam ElasticModelTemplate
 */
template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute_CPU final
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using elem_stress_t = List<math::matr<dim, dim>>;
  using vert_stress_t = math::fieldr<dim>;
  using elem_hessian_t = List<math::matr<dim * dim, dim * dim>>;
  using vert_hessian_t = math::spmatr;
  using MeshPtr = SPtr<TriMesh<dim>>;
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  ElasticityCompute_CPU(SPtr<TriMesh<dim>> mesh);
  ~ElasticityCompute_CPU() override;

  void UpdateEnergy() final;
  void UpdateStress() final;
  void UpdateHessian(bool projection) final;

  void GatherEnergyToVertices() final;
  void GatherStressToVertices() final;
  void GatherHessianToVertices() final;
  bool Update(math::fieldr<dim> const& pose, ElasticityUpdateLevel update_type) final;
  void RecomputeRestPose() final;

protected:
  List<math::decomp::SvdResult<dim, real>> svd_results_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
  struct TbbPartitioners;  ///< include tbb in CUDA will cause error, so we have to put it here.
  std::unique_ptr<TbbPartitioners> partitioner_impl_;
};

}  // namespace ax::fem
