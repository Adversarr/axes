#include "test_problems.hpp"

#include <iostream>

#include "ax/core/logging.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linsys/sparse/LDLT.hpp"
#include "ax/math/linsys/sparse/QR.hpp"
#include "ax/optim/spsdm/eigenvalue.hpp"

namespace ax::optim::test {

/************************* SECT: RosenbrockProblem *************************/
Real rosenbrock(Variable const& x_mat) {
  math::RealVectorX x = x_mat;
  Real f = 0;
  for (Index i = 0; i < x.size() - 1; ++i) {
    f += math::square(x[i] - 1) + 100 * math::square(x[i + 1] - math::square(x[i]));
  }
  return f;
}

Gradient rosenbrock_grad(Variable const& x_mat) {
  math::RealVectorX x = x_mat;
  math::RealVectorX grad(x.size());
  for (Index i = 0; i < x.size() - 1; ++i) {
    grad[i] = -2 * (1 - x[i]) - 400 * x[i] * (x[i + 1] - math::square(x[i]));
  }
  grad[x.size() - 1] = 200 * (x[x.size() - 1] - math::square(x[x.size() - 2]));
  return grad;
}

DenseHessian rosenbrock_hessian(Variable const& x_mat) {
  math::RealVectorX x = x_mat;
  math::RealMatrixX hessian = math::zeros(x.size(), x.size());
  for (Index i = 0; i < x.size() - 1; ++i) {
    hessian(i, i) = 2 - 400 * x[i + 1] + 1200 * math::square(x[i]);
    hessian(i, i + 1) = -400 * x[i];
    hessian(i + 1, i) = -400 * x[i];
  }
  hessian(x.size() - 1, x.size() - 1) = 200;

  EigenvalueModification modification;
  modification.min_eigval_ = 1e-3;
  auto spsd_hessian = modification.Modify(hessian);
  return spsd_hessian;
}

Variable RosenbrockProblem::Optimal(Variable const& x0) {
  return math::RealVectorX::Ones(x0.size());
}

RosenbrockProblem::RosenbrockProblem() {
  SetEnergy(rosenbrock);
  SetGrad(rosenbrock_grad);
  SetHessian(rosenbrock_hessian);
}

/************************* SECT: Least Square *************************/
LeastSquareProblem::LeastSquareProblem(DenseHessian const& A, Variable const& b) : A_(A), b_(b) {
  SetEnergy([this](Variable const& x) -> Real {
    return (0.5 * (x - b_).transpose() * A_ * (x - b_))(0, 0);
  });
  SetGrad([this](Variable const& x) {
    return A_ * (x - b_);
  });
  SetHessian([this](Variable const&) {
    return A_;
  });
}

Variable LeastSquareProblem::Optimal(Variable const&) {
  return b_;
}

SparseLeastSquareProblem::SparseLeastSquareProblem(SparseHessian const& A, Variable const& b)
    : A_(A), b_(b) {
  AX_CHECK(A.rows() == A.cols(), "A must be square");
  AX_CHECK(A.rows() == b.rows(), "A and b must have the same rows");
  SetEnergy([this](Variable const& x) -> Real {
    Variable residual = x - b_;
    return 0.5 * math::inner(residual.reshaped(), A_, residual.reshaped());
  });
  SetGrad([this](Variable const& x) -> Gradient {
    return A_ * (x - b_);
  });
  SetSparseHessian([this](Variable const&) -> SparseHessian {
    return A_;
  });
}

Variable SparseLeastSquareProblem::Optimal(Variable const& x0) {
  return b_;
}

static auto prepare_laplace(Index n) {
  math::SparseCOO coo;
  for (Index i = 0; i < n; ++i) {
    coo.push_back({i, i, 2});
    if (i > 0) {
      coo.push_back({i, i - 1, -1});
    }
    if (i < n - 1) {
      coo.push_back({i, i + 1, -1});
    }
  }
  math::RealSparseMatrix laplace = math::make_sparse_matrix(n, n, coo);
  return laplace;
}

PoissonProblem::PoissonProblem(Index n)
    : SparseLeastSquareProblem(prepare_laplace(n), Variable::Random(n, 1)) {
}

}  // namespace ax::optim::test
