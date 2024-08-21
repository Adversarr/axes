#pragma once
#include "ax/optim/spsdm.hpp"
#include "ax/math/linalg.hpp"
namespace ax::optim {

class EigenvalueModification : public SpsdModificationBase {
public:
  virtual math::RealMatrixX Modify(math::RealMatrixX const& A) final;
  virtual math::RealSparseMatrix Modify(math::RealSparseMatrix const& A) final;

  void SetOptions(utils::Options const& options) final;
  utils::Options GetOptions() const final;

  Real min_eigval_{1e-6};
};

template<int dim>
AX_HOST_DEVICE AX_FORCE_INLINE math::RealMatrix<dim, dim>
project_spd_by_eigvals(math::RealMatrix<dim, dim> const& A, Real min_eigval) {
  math::RealMatrix<dim, dim> V;
  math::RealVector<dim> D;
  math::eig(A, V, D);
  D = D.cwiseMax(min_eigval);
  return V * D.asDiagonal() * V.transpose();
}


}  // namespace ax::optim
