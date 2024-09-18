#pragma once
#include "ax/fem/elasticity/compute.hpp"
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"
#include "ax/math/high_order/gather.hpp"

namespace ax::fem {

class ElasticityTerm : public TermBase {
public:
  ElasticityTerm(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh);

  void UpdateEnergy() override;

  void UpdateGradient() override;

  void UpdateHessian() override;
  // TODO: options.

private:
  ElasticityKind kind_{ElasticityKind::Linear};

  // Static
  BufferPtr<Real> rest_;         // (dim, nE). Rest position of each element.
  BufferPtr<Real> rest_volume_;  // (nE, ). Rest volume of each element.
  BufferPtr<Real> dminv_;        // (dim, dim, nE). See "Dynamic Deformables", map x->F
  BufferPtr<Real> pfpx_;         // (dim * dim, dim * (dim + 1), nE). derivative of Deformation Gradient wrt x.

  ElasticityBatchedCompute compute_;  // Compute the energy, gradient, and hessian.

  // Runtimes
  BufferPtr<Real> elem_grad_; // (dim, nVPE, nE) gradient of the energy wrt element.
  BufferPtr<Real> elem_hess_; // (dim, dim, nVPE * nVPE * nE) hessian of the energy wrt element.
  math::GatherAddOp gather_hessian_;   // gather the local hessian into the global bsr.
  math::GatherAddOp gather_gradient_;  // gather the local gradient into the global gradient.
  // ... many other buffers are declared in TermBase
};

}  // namespace ax::fem