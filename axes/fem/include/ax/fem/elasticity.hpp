#pragma once
#include "ax/math/sparse.hpp"
#include "ax/utils/enum_refl.hpp"
#include "elasticity/common.hpp"
#include "trimesh.hpp"

namespace ax::fem {

AX_DEFINE_ENUM_CLASS(ElasticityUpdateLevel, kEnergy, kStress, kHessian);

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  using elem_stress_t = std::vector<math::matr<dim, dim>>;
  using vert_stress_t = math::fieldr<dim>;
  using elem_hessian_t = std::vector<math::matr<dim * dim, dim * dim>>;
  using vert_hessian_t = math::spmatr;
  using MeshPtr = std::shared_ptr<TriMesh<dim>>;

  explicit ElasticityComputeBase(std::shared_ptr<TriMesh<dim>> mesh);
  virtual ~ElasticityComputeBase() = default;

  void SetMesh(MeshPtr const& mesh);
  virtual void RecomputeRestPose();

  virtual void SetLame(math::vec2r const& u_lame);
  virtual void SetLame(math::field2r const& e_lame);

  virtual void Update(math::fieldr<dim> const& pose, ElasticityUpdateLevel update_type) = 0;

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
  std::shared_ptr<TriMesh<dim>> mesh_;
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

}  // namespace ax::fem
