#pragma once
#include <tbb/partitioner.h>

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
  using vert_hessian_t = math::sp_matxxr;
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

  // SECT: Old APIs.
  virtual math::field1r GatherEnergy(math::field1r const& element_values) const;
  virtual math::fieldr<dim> GatherStress(List<elasticity::StressTensor<dim>> const& stress) const;
  virtual math::sp_matxxr GatherHessian(List<elasticity::HessianTensor<dim>> const& hessian) const;
  virtual math::field1r ComputeEnergyAndGather(math::vec2r const& u_lame);
  virtual math::field1r ComputeEnergyAndGather(math::field2r const& lame);
  virtual math::fieldr<dim> ComputeStressAndGather(math::vec2r const& u_lame);
  virtual math::fieldr<dim> ComputeStressAndGather(math::field2r const& lame);
  virtual math::sp_matxxr ComputeHessianAndGather(math::vec2r const& u_lame);
  virtual math::sp_matxxr ComputeHessianAndGather(math::field2r const& lame);

  virtual math::field1r Energy(math::vec2r const& u_lame) = 0;
  virtual math::field1r Energy(math::field2r const& lame) = 0;

  virtual List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) = 0;
  virtual List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) = 0;

  virtual List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) = 0;
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) = 0;

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
  using vert_hessian_t = math::sp_matxxr;
  using MeshPtr = SPtr<TriMesh<dim>>;
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  virtual void UpdateEnergy();
  virtual void UpdateStress();
  virtual void UpdateHessian(bool projection);

  virtual void GatherEnergyToVertices();
  virtual void GatherStressToVertices();
  virtual void GatherHessianToVertices();

  bool Update(math::fieldr<dim> const& pose, ElasticityUpdateLevel update_type);
  void RecomputeRestPose();
  math::field1r Energy(math::field2r const& lame);
  math::field1r Energy(math::vec2r const& lame);
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame);
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame);

protected:
  List<math::decomp::SvdResultImpl<dim, real>> svd_results_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
  tbb::affinity_partitioner e_ap, s_ap, h_ap, svd_ap;
};

}  // namespace ax::fem
