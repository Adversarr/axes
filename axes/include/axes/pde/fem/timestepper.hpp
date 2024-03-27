#pragma once
#include "axes/math/common.hpp"
#include "axes/pde/fem/deform.hpp"
#include "axes/pde/fem/elasticity.hpp"
#include "axes/pde/fem/mesh.hpp"

namespace ax::pde::fem {

template <idx dim>
class TimeStepperBase : public utils::Tunable {
public:
  // Constructors and destructors
  TimeStepperBase(UPtr<MeshBase<dim>> mesh);
  AX_DECLARE_CONSTRUCTOR(TimeStepperBase, delete, delete);
  virtual ~TimeStepperBase() = default;

  // Accessors
  MeshBase<dim> const &GetMesh() const { return *mesh_; }
  Deformation<dim> const &GetDeformation() const { return *deform_; }
  ElasticityComputeBase<dim> const &GetElasticity() const { return *elasticity_; }

  // Do the time stepping.
  virtual Status Step(real dt=0.01);
  virtual Status Init(utils::Opt const& opt = {});

private:
  UPtr<MeshBase<dim>> mesh_;
  math::fieldr<dim> velocity_;
  UPtr<Deformation<dim>> deform_;
  UPtr<ElasticityComputeBase<dim>> elasticity_;
};

}