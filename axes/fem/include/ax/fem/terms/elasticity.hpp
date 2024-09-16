#pragma once
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"
#include "ax/math/high_order/gather.hpp"

namespace ax::fem {

AX_DEFINE_ENUM_CLASS(ElasticityKind, Linear, IsotropicARAP, StVK, NeoHookean, StableNeoHookean, );

class ElasticityTerm : public TermBase {
public:
  ElasticityTerm(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh);

  Real Energy() const override;

  void UpdateGradient() override;

  void UpdateHessian() override;

  // TODO: options.

private:
  ElasticityKind kind_{ElasticityKind::Linear};

  // Static
  BufferPtr<Real> lame_;         // (2, nE). Lame parameters.
  BufferPtr<Real> rest_;         // (dim, nE). Rest position of each element.
  BufferPtr<Real> rest_volume_;  // (nE, ). Rest volume of each element.
  BufferPtr<Real> deform_grad_;  // (dim, dim, nE). Deformation gradient of each element.
  BufferPtr<Real> dminv_;        // (dim, dim, nE). See "Dynamic Deformables", map x->F
  BufferPtr<Real> pfpx_;         // (?, ?, nE). derivative of Deformation Gradient wrt x.

  // Runtimes
  BufferPtr<Real> energy_;         // (nE, ). Energy of each element, with density multiplied.
  BufferPtr<Real> pk1_;            // (dim, dim, nE). First-Piola Kirchhoff stress on each element
                                   // also the (partial Energy/partial F)
  BufferPtr<Real> local_hessian_;  // (nVpe*dim, nVpe*dim, nE). Local Hessian of each element.
  math::GatherAddOp gather_hessian_;   // gather the local hessian into the global bsr.
  math::GatherAddOp gather_gradient_;  // gather the local gradient into the global gradient.
  // ... many other buffers are declared in TermBase
};

}  // namespace ax::fem