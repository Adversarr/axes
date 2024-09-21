#pragma once

#include "ax/core/gsl.hpp"
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"
#include "ax/fem/terms/elasticity.hpp"
#include "ax/fem/terms/mass.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

class TimeStepBase : public utils::Tunable {
public:
  explicit TimeStepBase(shared_not_null<Mesh> mesh);
  ~TimeStepBase() override = default;

  not_null<Problem> GetProblem() const;

  // The inertia part.
  not_null<MassTerm *> GetInertia() const;

  // The elastic part.
  not_null<ElasticityTerm *> GetElasticity() const;

  void SetOptions(const utils::Options &option) override;

  // Helper function to set the elasticity.
  void SetElasticity(ElasticityKind kind);

  // Helper function to set the lame.
  void SetLame(ConstRealBufferView lame);
  void SetLame(const math::RealVector2 &lame);  // uniform version

  // Helper function to set the density.
  void SetDensity(ConstRealBufferView density);
  void SetDensity(Real density);  // uniform version

  // Helper function to set the external force.
  void SetExternalAcceleration(ConstRealBufferView ext_force);
  void SetExternalAcceleration(const math::RealVector2 &ext_force);  // uniform version
  void SetExternalAcceleration(const math::RealVector3 &ext_force);  // uniform version

  // Helper function, to update the Problem's energy, gradient, and hessian.
  void UpdateEnergy();
  void UpdateGradient();
  void UpdateHessian();

  void StepToNext();

  virtual void SolveStep();

private:
  Real dt_;                        ///< The timestep.
  BufferPtr<Real> velocity_;       ///< The velocity of each vertex
  BufferPtr<Real> u_back_;         ///< The last displacement of each vertex
  BufferPtr<Real> velocity_back_;  ///< The last velocity of each vertex
  BufferPtr<Real> u_;              ///< The displacement of each vertex,
                                   ///< should be the same as the problem_.state_.variable_
  Problem problem_;                ///< The timestep problem.
  shared_not_null<Mesh> mesh_;     ///< The mesh.
};

}  // namespace ax::fem