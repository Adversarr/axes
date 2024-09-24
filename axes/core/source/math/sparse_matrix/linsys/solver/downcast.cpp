#include "ax/math/sparse_matrix/linsys/solver/downcast.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"

namespace ax::math {

GeneralSparseSolver_Downcast::GeneralSparseSolver_Downcast() {
#ifdef AX_HAS_CHOLMOD
  host_solver_ = HostSparseSolverBase::Create(HostSparseSolverKind::Cholmod);
#else
  host_solver_ = HostSparseSolverBase::Create(HostSparseSolverKind::LDLT);
#endif
}

void GeneralSparseSolver_Downcast::AnalyzePattern() {
}

void GeneralSparseSolver_Downcast::Factorize() {
  auto host_mat = mat_->Transfer(BufferDevice::Host)->ToSparseMatrix();
  host_solver_->SetProblem(host_mat).Compute();
}


BlockedLinsysSolveStatus GeneralSparseSolver_Downcast::Solve(ConstRealBufferView b, RealBufferView x) const {
  host_b_.resize(prod(b.Shape()));
  host_x_.resize(prod(x.Shape()));

  auto hb = view_from_matrix(host_b_).Reshaped(b.Shape());
  auto hx = view_from_matrix(host_x_).Reshaped(x.Shape());

  copy(hb, b);
  copy(hx, x);

  auto result = host_solver_->Solve(host_b_, host_x_);
  copy(x, view_from_matrix(result.solution_).Reshaped(x.Shape()));
  BlockedLinsysSolveStatus status;
  status.converged_ = result.converged_;
  status.iter_ = result.num_iter_;
  status.l2_err_ = result.l2_err_;

  return status;
}

}