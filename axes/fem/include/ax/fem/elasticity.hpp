#pragma once
#include "ax/math/sparse.hpp"
#include "ax/utils/enum_refl.hpp"
#include "elasticity/common.hpp"
#include "trimesh.hpp"

namespace ax::fem {

AX_DEFINE_ENUM_CLASS(ElasticityUpdateLevel, Energy, Stress, Hessian);

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <int dim> class ElasticityComputeBase {
public:
  using elem_stress_t = math::aligned_vector<math::RealMatrix<dim, dim>>;
  using vert_stress_t = math::RealField<dim>;
  using elem_hessian_t = math::aligned_vector<math::RealMatrix<dim * dim, dim * dim>>;
  using vert_hessian_t = math::RealSparseMatrix;
  using MeshPtr = std::shared_ptr<TriMesh<dim>>;

  explicit ElasticityComputeBase(std::shared_ptr<TriMesh<dim>> mesh);
  virtual ~ElasticityComputeBase() = default;

  void SetMesh(MeshPtr const& mesh);
  virtual void RecomputeRestPose();

  virtual void SetLame(math::RealVector2 const& u_lame);
  virtual void SetLame(math::RealField2 const& e_lame);

  virtual void Update(math::RealField<dim> const& pose, ElasticityUpdateLevel update_type) = 0;

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
  virtual math::RealField1 const& GetEnergyOnElements() { return energy_on_elements_; }
  virtual math::RealField1 const& GetEnergyOnVertices() { return energy_on_vertices_; }

protected:
  std::shared_ptr<TriMesh<dim>> mesh_;
  elasticity::DeformGradCache<dim> rinv_;
  math::RealField1 rest_volume_;
  math::RealField2 lame_;

  math::RealField1 energy_on_elements_;
  math::RealField1 energy_on_vertices_;
  elem_stress_t stress_on_elements_;
  vert_stress_t stress_on_vertices_;
  elem_hessian_t hessian_on_elements_;
  vert_hessian_t hessian_on_vertices_;
};

}  // namespace ax::fem
