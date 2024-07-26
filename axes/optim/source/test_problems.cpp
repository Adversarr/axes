#include "ax/optim/test_problems.hpp"

#include "ax/core/logging.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/linsys/sparse/QR.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"

namespace ax::optim::test {

/************************* SECT: RosenbrockProblem *************************/
real rosenbrock(math::vecxr const& x) {
  real f = 0;
  for (idx i = 0; i < x.size() - 1; ++i) {
    f += math::square(x[i] - 1) + 100 * math::square(x[i + 1] - math::square(x[i]));
  }
  return f;
}

math::vecxr rosenbrock_grad(math::vecxr const& x) {
  math::vecxr grad(x.size());
  for (idx i = 0; i < x.size() - 1; ++i) {
    grad[i] = -2 * (1 - x[i]) - 400 * x[i] * (x[i + 1] - math::square(x[i]));
  }
  grad[x.size() - 1] = 200 * (x[x.size() - 1] - math::square(x[x.size() - 2]));
  return grad;
}

math::matxxr rosenbrock_hessian(math::vecxr const& x) {
  math::matxxr hessian = math::zeros(x.size(), x.size());
  for (idx i = 0; i < x.size() - 1; ++i) {
    hessian(i, i) = 2 - 400 * x[i + 1] + 1200 * math::square(x[i]);
    hessian(i, i + 1) = -400 * x[i];
    hessian(i + 1, i) = -400 * x[i];
  }
  hessian(x.size() - 1, x.size() - 1) = 200;

  optim::EigenvalueModification modification;
  modification.min_eigval_ = 1e-3;
  auto spsd_hessian = modification.Modify(hessian);
  return spsd_hessian;
}

math::vecxr RosenbrockProblem::Optimal(math::vecxr const& x0) {
  return math::vecxr::Ones(x0.size());
}

RosenbrockProblem::RosenbrockProblem() {
  SetEnergy(rosenbrock);
  SetGrad(rosenbrock_grad);
  SetHessian(rosenbrock_hessian);
}

/************************* SECT: Least Square *************************/
LeastSquareProblem::LeastSquareProblem(math::matxxr const& A, math::vecxr const& b) : A_(A), b_(b) {
  SetEnergy([this](math::vecxr const& x) { return 0.5 * (x - b_).transpose() * A_ * (x - b_); });
  SetGrad([this](math::vecxr const& x) { return A_ * (x - b_); });
  SetHessian([this](math::vecxr const&) { return A_; });
}

math::vecxr LeastSquareProblem::Optimal(math::vecxr const&) { return b_; }

SparseLeastSquareProblem::SparseLeastSquareProblem(math::spmatr const& A, math::vecxr const& b)
    : A_(A), b_(b) {
  AX_CHECK(A.rows() == A.cols());
  AX_CHECK(A.rows() == b.rows());
  SetEnergy([this](math::vecxr const& x) {
    AX_CHECK(x.rows() == b_.rows()) << "x.rows() = " << x.rows() << ", b_.rows() = " << b_.rows();
    math::vecxr residual = A_ * x - b_;
    return 0.5 * residual.dot(residual);
  });
  SetGrad([this](math::vecxr const& x) {
    AX_CHECK(x.rows() == b_.rows()) << "x.rows() = " << x.rows() << ", b_.rows() = " << b_.rows();
    return A_.transpose() * (A_ * x - b_);
  });
  SetSparseHessian([this](math::vecxr const& x) {
    AX_CHECK(x.rows() == b_.rows()) << "x.rows() = " << x.rows() << ", b_.rows() = " << b_.rows();
    return A_.transpose() * A_;
  });
}

math::vecxr SparseLeastSquareProblem::Optimal(math::vecxr const&) {
  math::vecxr x_opt = math::vecxr::Zero(b_.rows());
  math::SparseSolver_QR solver;
  solver.SetProblem(A_).Compute();
  auto solution = solver.Solve(b_, {});
  x_opt = solution.solution_;
  return x_opt;
}

}  // namespace ax::optim::test
