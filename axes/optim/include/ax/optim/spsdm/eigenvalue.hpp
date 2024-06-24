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

template<idx dim>
AX_HOST_DEVICE AX_FORCE_INLINE math::matr<dim, dim>
project_spd_by_eigvals(math::matr<dim, dim> const& A, real min_eigval) {
  math::matr<dim, dim> V;
  math::vecr<dim> D;
  math::eig(A, V, D);
  D = D.cwiseMax(min_eigval);
  return V * D.asDiagonal() * V.transpose();
}


}  // namespace ax::optim
