#pragma once
#include "ax/optim/spsdm.hpp"
#include "ax/math/linalg.hpp"
namespace ax::optim {

class EigenvalueModification : public SpsdModificationBase {
public:
  virtual math::matxxr Modify(math::matxxr const& A) final;
  virtual math::spmatr Modify(math::spmatr const& A) final;

  void SetOptions(utils::Options const& options) final;
  utils::Options GetOptions() const final;

  real min_eigval_{1e-6};
};

template<Index dim>
AX_HOST_DEVICE AX_FORCE_INLINE math::RealMatrix<dim, dim>
project_spd_by_eigvals(math::RealMatrix<dim, dim> const& A, real min_eigval) {
  math::RealMatrix<dim, dim> V;
  math::RealVector<dim> D;
  math::eig(A, V, D);
  D = D.cwiseMax(min_eigval);
  return V * D.asDiagonal() * V.transpose();
}


}  // namespace ax::optim
