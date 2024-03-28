#pragma once
#include "axes/math/common.hpp"
#include "axes/pde/fem/deform.hpp"
#include "axes/pde/fem/elasticity.hpp"
#include "axes/pde/fem/mesh.hpp"
#include "axes/utils/opt.hpp"

namespace ax::pde::fem {

// Use Backward Euler:
//     M (x' - (x + dt v)) = dt^2 F_int(x')
// Apply 2nd Taylor expansion to approximate F_int(x') = F_int(x) + K (x' - x) + O(dx^2)
//    (M + dt^2 K) x' = M (x + dt v) + dt^2 F_int(x)
// where velocity = (x - x_{n-1})/dt
// This gives us a linear system to solve:
//    (M + dt^2 K) (x' - x) = dt M v + dt^2 F_int(x)
// Because we use Linear model, K is constant, and the approximation is exact.
// One step of Linear solve should give you the correct result.
template <idx dim>
class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase(UPtr<MeshBase<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, delete, delete);
  virtual ~TimeStepperBase() = default;

  // Accessors
  MeshBase<dim> &GetMesh() { return *mesh_; }
  Deformation<dim> &GetDeformation() { return *deform_; }
  ElasticityComputeBase<dim> &GetElasticity() { return *elasticity_; }

  // Do the time stepping.
  virtual Status Step(real dt=0.01);
  virtual Status Init(utils::Opt const& opt = {});

private:
  UPtr<MeshBase<dim>> mesh_;
  UPtr<Deformation<dim>> deform_;
  UPtr<ElasticityComputeBase<dim>> elasticity_;

  math::sp_matxxr mass_matrix_;
  math::fieldr<dim> ext_force_;
  math::fieldr<dim> velocity_;
};

}