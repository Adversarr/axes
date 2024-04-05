#pragma once
#include "ax/math/common.hpp"
#include "ax/utils/opt.hpp"
#include "deform.hpp"
#include "elasticity.hpp"
#include "mesh_base.hpp"

namespace ax::fem {

// Use Backward Euler:
//     M (x' - (x + dt v)) = dt^2 F_int(x')
// Apply 2nd Taylor expansion to approximate F_int(x') = F_int(x) + K (x' - x) + O(dx^2)
//    (M + dt^2 K) x' = M (x + dt v) + dt^2 F_int(x)
// where velocity = (x - x_{n-1})/dt
// This gives us a linear system to solve:
//    (M + dt^2 K) (x' - x) = dt M v + dt^2 F_int(x)
// Because we use Linear model, K is constant, and the approximation is exact.
// One step of Linear solve should give you the correct result.
template <idx dim> class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase(UPtr<MeshBase<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, delete, default);
  virtual ~TimeStepperBase() = default;

  // Accessors
  MeshBase<dim> &GetMesh() { return *mesh_; }
  Deformation<dim> &GetDeformation() { return *deform_; }
  ElasticityComputeBase<dim> &GetElasticity() { return *elasticity_; }

  // External force getter/setter
  math::fieldr<dim> const &GetExternalAcceleration() const { return ext_accel_; }
  void SetExternalAcceleration(math::fieldr<dim> const &ext_force) { ext_accel_ = ext_force; }

  // Do the time stepping.
  virtual Status Step(real dt = 0.01);
  virtual Status Init(utils::Opt const &opt = {});

  // Setters
  template <template <idx> class ElasticModelTemplate> void SetupElasticity() {
    elasticity_ = std::make_unique<ElasticityCompute<dim, ElasticModelTemplate>>(*deform_);
  }

  void SetLame(math::vec2r const &lame) { lame_ = lame; }
  math::vec2r const &GetLame() const { return lame_; }

  void SetDensity(real density);

  void SetDensity(math::field1r const &density, bool is_density_on_elements=false);

protected:
  // Common data
  UPtr<MeshBase<dim>> mesh_;
  UPtr<Deformation<dim>> deform_;
  UPtr<ElasticityComputeBase<dim>> elasticity_;
  math::fieldr<dim> ext_accel_;
  math::fieldr<dim> velocity_;
  math::vec2r lame_;

  // cache the mass matrix
  math::sp_matxxr mass_matrix_;
};

}  // namespace ax::fem