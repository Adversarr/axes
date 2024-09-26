#pragma once
#include "ax/fem/elasticity/compute.hpp"
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"
#include "ax/math/high_order/gather.hpp"

namespace ax::fem {

class ElasticityTerm : public TermBase {
public:
  ElasticityTerm(shared_not_null<State> state, shared_not_null<Mesh> mesh);

  void UpdateEnergy() override;

  void UpdateGradient() override;

  void UpdateHessian() override;
  // TODO: options.

  void SetKind(ElasticityKind kind);
  ElasticityKind GetKind() const noexcept;

  void SetLame(ConstRealBufferView lame);

  void SetHessianMakeSPSD(bool make_spsd);

// private:
  // Static
  BufferPtr<Real> rest_;         // (dim, nE). Rest position of each element.
  BufferPtr<Real> rest_volume_;  // (nE, ). Rest volume of each element.
  BufferPtr<Real> dminv_;        // (dim, dim, nE). See "Dynamic Deformables", map x->F
  BufferPtr<Real> pfpx_;         // (dim * dim, dim * (dim + 1), nE). derivative of DeformGrad wrt x.

  ElasticityBatchedCompute compute_;  // Compute the energy, gradient, and hessian.

  // Runtimes
  BufferPtr<Real> elem_grad_; // (dim, nVPE, nE) gradient of the energy wrt element.
  BufferPtr<Real> elem_hess_; // (dim, dim, nVPE * nVPE * nE) hessian of the energy wrt element.
  math::GatherAddOp gather_hessian_;   // gather the local hessian into the global bsr.
  math::GatherAddOp gather_gradient_;  // gather the local gradient into the global gradient.
  // ... many other buffers are declared in TermBase.
};

}  // namespace ax::fem