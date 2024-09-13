#include "ax/math/block_matrix/linsys/solver/cg.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_default.hpp"
#include "ax/math/block_matrix/linsys/preconditioner.hpp"
#include "ax/math/buffer_blas.hpp"

namespace ax::math {

BlockedLinsysSolveStatus BlockSolver_ConjugateGradient::Solve(ConstRealBufferView b,
                                                              RealBufferView x) const {
  // Perform a block based CG.
  const auto device = problem_->A_.GetDevice();
  AX_THROW_IF_FALSE(is_2d(b.Shape()), "rhs should be 2D");
  AX_THROW_IF_FALSE(is_2d(x.Shape()), "x should be 2D");
  AX_THROW_IF_FALSE(b.Shape() == x.Shape(), "rhs must be same shape with x");

  auto residual_buf = create_buffer<Real>(device, b.Shape());

  auto p_buf = create_buffer<Real>(device, x.Shape());
  auto d_buf = create_buffer<Real>(device, x.Shape());
  auto q_buf = create_buffer<Real>(device, x.Shape());  // q = A d
  auto [r, p, d, q] = make_view(residual_buf, p_buf, d_buf, q_buf);
  const auto& mata = problem_->A_;

  // r <- b-Ax
  buffer_blas::copy(r, b);
  mata.RightMultiplyTo(x, r, -1, 1);

  if (preconditioner_) {
    preconditioner_->Solve(r, d);  // d <- M^-1 r
  } else {
    buffer_blas::copy(d, r);  // d <- r
  }

  Real delta_new = buffer_blas::dot(r, d);  // delta_new <- r.T d

  size_t iter = 0;
  BlockedLinsysSolveStatus status;
  for (; iter < max_iter_; ++iter) {
    // q <- A d
    mata.RightMultiplyTo(d, q);

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

    // check convergence
    status.l2_err_ = buffer_blas::norm(r);
    if (status.l2_err_ <= problem_->l2_tol_) {
      status.converged_ = true;
      break;
    }
    AX_INFO("CG iter: {}, l2_err: {}", iter, status.l2_err_);
  }

  status.iter_ = iter;
  return status;
}

}  // namespace ax::math