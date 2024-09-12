#include "ax/math/block_matrix/linsys/solver/cg.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_default.hpp"
#include "ax/math/block_matrix/linalg.hpp"
#include "ax/math/block_matrix/linsys/preconditioner.hpp"

namespace ax::math {

void BlockSolver_ConjugateGradient::AnalyzePattern() {
  if (preconditioner_) {
    preconditioner_->AnalyzePattern();
  }
}

void BlockSolver_ConjugateGradient::Factorize() {
  if (preconditioner_) {
    preconditioner_->Factorize();
  }
}

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
  block_blas::copy(r, b);
  mata.RightMultiplyTo(x, r, -1, 1);

  if (preconditioner_) {
    preconditioner_->Solve(r, d);  // d <- M^-1 r
  } else {
    block_blas::copy(p, d);  // p <- y.
  }

  Real delta_new = block_blas::dot(r, d);  // delta_new <- r.T d

  size_t iter = 0;
  BlockedLinsysSolveStatus status;
  for (; iter < max_iter_; ++iter) {
    // q <- A d
    mata.RightMultiplyTo(d, q);

    // alpha <- delta_new / d^T q
    Real d_dot_q = block_blas::dot(d, q);
    Real alpha = delta_new / d_dot_q;

    // x <- x + alpha d
    block_blas::axpy(alpha, d, x);

    // r <- r - alpha q
    block_blas::axpy(-alpha, q, r);

    // s = M^{-1} r, but q is not used any more in this iteration
    // we reuse q buffer.
    if (preconditioner_) {
      preconditioner_->Solve(r, q);
    } else {
      block_blas::copy(q, r);  // s <- r
    }

    // delta_old <- delta_new
    Real delta_old = delta_new;
    delta_new = block_blas::dot(r, q);  // delta_new <- r.T s
    Real beta = delta_new / delta_old;
    block_blas::scal(beta, d);  // d <- beta d
    block_blas::axpy(1, q, d);  // d <- s + beta d

    // check convergence
    status.l2_err_ = block_blas::norm(r);
    if (status.l2_err_ <= problem_->l2_tol_) {
      status.converged_ = true;
      break;
    }
    status.linf_err_ = block_blas::amax(r);
    if (status.linf_err_ <= problem_->linf_tol_) {
      status.converged_ = true;
      break;
    }
  }

  status.iter_ = iter;
  return status;
}

}  // namespace ax::math