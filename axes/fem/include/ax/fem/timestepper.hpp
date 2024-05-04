#pragma once
#include "ax/math/common.hpp"
#include "ax/utils/opt.hpp"
#include "elasticity.hpp"
#include "trimesh.hpp"

namespace ax::fem {

class TimeStepConfig {
public:
  real dt_;
};

// Use Backward Euler (with exactly 1 step backward)
//     M (x' - (x + dt v)) = dt^2 F_int(x')
// Apply 2nd Taylor expansion to approximate F_int(x') = F_int(x) + K (x' - x) + O(dx^2)
//    (M + dt^2 K) x' = M (x + dt v) + dt^2 F_int(x)
// where velocity = (x - x_{n-1})/dt
// This gives us a linear system to solve:
//    (M + dt^2 K) (x' - x) = dt M v + dt^2 F_int(x)
// Because we use Linear model, K is constant, and the approximation is exact.
// One step of Linear solve should give you the correct result.
// WARNING: You should not apply the base solver to solve real world problems, this implementation
//          is only reserved for testing.
template <idx dim> class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase(SPtr<TriMesh<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, delete, default);
  virtual ~TimeStepperBase() = default;

  // Accessors
  TriMesh<dim> &GetMesh() { return *mesh_; }
  SPtr<TriMesh<dim>> &GetMeshPtr() { return mesh_; }
  ElasticityComputeBase<dim> &GetElasticity() { return *elasticity_; }

  // External force getter/setter
  math::fieldr<dim> const &GetExternalAcceleration() const { return ext_accel_; }
  void SetExternalAcceleration(math::fieldr<dim> const &ext_force) { ext_accel_ = ext_force; }

  // Do the time stepping.
  // TODO: dt should be removed.
  virtual Status Step(real dt);

  // Setters
  virtual Status Init(utils::Opt const &opt = {});

  // Overide the default elasticity.
  template <template <idx> class ElasticModelTemplate, 
            template <idx, template <idx> class> class Compute = ElasticityCompute_CPU> void SetupElasticity() {
    if (mesh_) {
      elasticity_ = std::make_unique<Compute<dim, ElasticModelTemplate>>(mesh_);
      elasticity_->RecomputeRestPose();
    }
  }

  void SetLame(math::vec2r const &lame) { u_lame_ = lame; }

  math::vec2r const &GetLame() const { return u_lame_; }

  void SetDensity(real density);

  void SetDensity(math::field1r const &density);

  math::sp_matxxr const &GetMassMatrix() const { return mass_matrix_; }

  math::sp_matxxr const &GetMassMatrixOriginal() const { return mass_matrix_original_; }

  math::sp_matxxr GetStiffnessMatrix(math::fieldr<dim> const& x, bool project = false) const;

  math::fieldr<dim> GetElasticForce(math::fieldr<dim> const& x) const;

  math::fieldr<dim> GetInertiaPosition(real dt) const;

  virtual Status Precompute();

protected:
  // Common data
  SPtr<TriMesh<dim>> mesh_;
  UPtr<ElasticityComputeBase<dim>> elasticity_;
  math::fieldr<dim> ext_accel_;
  math::fieldr<dim> velocity_;
  math::vec2r u_lame_;

  // TODO: add switches for BDF-1, BDF-2, etc.

  // cache the mass matrix
  math::sp_matxxr mass_matrix_;
  math::sp_matxxr mass_matrix_original_;
  real dt_{1e-3};  ///< The time step, should be formulated here because many initializers use dt
                   ///< such as Quasi Newton proposed in Liu17.
};

}  // namespace ax::fem
