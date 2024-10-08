#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

void GeneralSparseSolver_ConjugateGradient::AnalyzePattern() {
  GeneralSparseSolverBase::AnalyzePattern();
  if (mat_->Cols() != mat_->Rows()) {
    AX_THROW_RUNTIME_ERROR("SparseSolver_ConjugateGradient: only support square matrix.");
  }

  const auto device = mat_->Device();
  p_buf_ = create_buffer<Real>(device, {mat_->Cols()});
  d_buf_ = create_buffer<Real>(device, {mat_->Cols()});
  q_buf_ = create_buffer<Real>(device, {mat_->Cols()});
  residual_buf_ = create_buffer<Real>(device, {mat_->Cols()});
}

BlockedLinsysSolveStatus GeneralSparseSolver_ConjugateGradient::Solve(ConstRealBufferView b,
                                                                      RealBufferView x) const {
  const auto device = mat_->Device();
  if (device != b.Device() || device != x.Device()) {
    AX_THROW_RUNTIME_ERROR(
        "SparseSolver_ConjugateGradient: this, b and x must be on the same device. got {} {} {}",
        device, b.Device(), x.Device());
  }
  AX_THROW_IF_FALSE(b.Shape() == x.Shape(), "rhs must be same shape with x");

  auto [r, p, d, q] = make_view(residual_buf_, p_buf_, d_buf_, q_buf_);
  r = r.Reshaped(b.Shape());
  p = p.Reshaped(b.Shape());
  d = d.Reshaped(b.Shape());
  q = q.Reshaped(b.Shape());

  const auto& mata = *mat_;

  // r <- b-Ax
  buffer_blas::copy(r, b);
  mata.Multiply(x, r, -1, 1);
  // if (zero_projection_) {
  //   zero_projection_(r);
  // }

  if (preconditioner_) {
    preconditioner_->Solve(r, d);  // d <- M^-1 r
    // if (zero_projection_) {
    //   zero_projection_(d);
    // }
  } else {
    buffer_blas::copy(d, r);  // d <- r
  }

  Real delta_new = buffer_blas::dot(r, d);  // delta_new <- r.T d

  BlockedLinsysSolveStatus status;
  size_t& iter = status.iter_;
  Real& err = status.l2_err_;
  bool& converged = status.converged_;
  for (; iter < max_iteration_; ++iter) {
    // check convergence
    err = buffer_blas::norm(r);
    // AX_INFO("CG iter: {}, l2_err: {:12.6e}", iter, err);
    if (err <= tolerance_) {
      converged = true;
      break;
    } else if (!math::isfinite(err)) {
      converged = false;
      AX_ERROR("CG diverged at iteration {}", iter);
      break;
    }

    // q <- A d
    mata.Multiply(d, q, 1, 0);

    // alpha <- delta_new / d^T q
    Real d_dot_q = buffer_blas::dot(d, q);
    Real alpha = delta_new / d_dot_q;

    // x <- x + alpha d
    buffer_blas::axpy(alpha, d, x);

    // r <- r - alpha q
    buffer_blas::axpy(-alpha, q, r);

    // s = M^{-1} r, but q is not used any more in this iteration
    // we reuse q buffer.
    if (preconditioner_) {
      preconditioner_->Solve(r, q);
    } else {
      buffer_blas::copy(q, r);  // s <- r
    }

    // delta_old <- delta_new
    Real delta_old = delta_new;
    delta_new = buffer_blas::dot(r, q);  // delta_new <- r.T s
    Real beta = delta_new / delta_old;
    buffer_blas::scal(beta, d);  // d' <- beta d
    buffer_blas::axpy(1, q, d);  // d  <- s + beta d

    // if (zero_projection_) {
    //   zero_projection_(q);
    // }
  }
  return status;
}

}  // namespace ax::math