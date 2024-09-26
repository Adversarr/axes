#include "ax/fem/timestepper/quasi_newton.hpp"

#include <exception>
#include <iostream>

#include "ax/core/entt.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/math/io.hpp"
#include "ax/math/linsys/preconditioner/FromLambda.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/optim/common.hpp"
#include "ax/utils/ndrange.hpp"
#include "ax/utils/opt.hpp"
#include "ax/utils/time.hpp"
// Very experimental
#define DIMENSION 128

using namespace ax;

bool apply_ppcg(math::RealSparseMatrix const &A, math::RealVectorX const &b, math::RealVectorX &x,
                math::RealVectorX &r, math::RealMatrixX const &z, Index max_iter = 1000,
                Real tol_l2 = 1e-6) {
  // Apply Projected Preconditioned Conjugate Gradient
  // 1. r = b - A x
  r = b - A * x;
  auto project_ortho_z = [&z](math::RealVectorX &v) {
    v = v - z * (z.transpose() * v);
  };
  {
    math::RealVectorX x_under_z = x;
    project_ortho_z(x_under_z);
    AX_INFO("PPCG: |x - Px|={:12.6e}", (x - x_under_z).norm());
  }
  math::RealVectorX g = r;
  project_ortho_z(g);  // g = P r
  math::RealVectorX d = -g;
  bool converged = false;
  Real r_dot_g = r.dot(g);
  for (Index i = 0; i < max_iter; ++i) {
    math::RealVectorX Ad = A * d;
    Real alpha = g.dot(g) / d.dot(Ad);
    x += alpha * d;
    r += alpha * Ad;
    project_ortho_z(r);
    math::RealVectorX g_next = r;
    project_ortho_z(g_next);
    Real r_dot_g_next = r.dot(g_next);
    Real beta = r_dot_g_next / r_dot_g;
    r_dot_g = r_dot_g_next;

    d = -g_next + beta * d;
    g = g_next;
    if (g.norm() < tol_l2) {
      converged = true;
      break;
    }

    AX_INFO("PPCG Iteration: {} |r|={:12.6e} |g|={:12.6e}", i, r.norm(), g.norm());
  }
  return converged;
}

bool apply_extended_cg(math::RealSparseMatrix const &A, math::RealVectorX const &b,
                       math::RealVectorX &x, math::RealVectorX &u, math::RealVectorX &r,
                       math::RealMatrixX const &z, Index max_iter = 1000, Real tol_l2 = 1e-6) {
  bool converged = false;

  // Our formula is:
  //    min. 1/2 |x + Z u - b|_A^2
  // the partial derivative is:
  //   partial x = A (x + Z u - b)
  //   partial u = Z^T A (x + Z u - b)
  // Use a CG to solve the above problem.
  // 1. r = A (x + Z u - b)
  u = math::RealVectorX::Zero(z.cols());
  r = A * (x + z * u - b);
  math::RealVectorX partial_x = r;
  math::RealVectorX partial_u = z.transpose() * r;

  math::RealVectorX p_x = -partial_x;
  math::RealVectorX p_u = -partial_u;
  Real rtr = r.dot(r);
  for (Index i = 0; i < max_iter; ++i) {
    math::RealVectorX Ap_x = A * p_x;
    math::RealVectorX Ap_u = z.transpose() * Ap_x;
    Real alpha
        = (partial_x.dot(partial_x) + partial_u.dot(partial_u)) / (p_x.dot(Ap_x) + p_u.dot(Ap_u));
    x += alpha * p_x;
    u += alpha * p_u;
    r += alpha * (Ap_x + z * Ap_u);
    partial_x = r;
    partial_u = z.transpose() * r;
    Real rtr_new = r.dot(r);
    Real beta = rtr_new / rtr;
    rtr_new = rtr;

    p_x = -partial_x + beta * p_x;
    p_u = -partial_u + beta * p_u;
    if (r.norm() < tol_l2) {
      converged = true;
      break;
    }
    AX_INFO("Extended CG Iteration: {} |r|={:12.6e}", i, r.norm());
  }
  return converged;
}

namespace ax::fem {
math::RealMatrixX basis_;

template <int dim>
void Timestepper_QuasiNewton<dim>::Initialize() {
  TimeStepperBase<dim>::Initialize();
  if (!solver_) {
    AX_WARN("Use default sparse solver: Cholmod.");
    solver_ = math::HostSparseSolverBase::Create(math::HostSparseSolverKind::Cholmod);
  }
}

template <int dim>
void Timestepper_QuasiNewton<dim>::UpdateSolverLaplace() {
  const auto &lame = this->lame_;
  Real dt = this->dt_;
  // TODO: If you are using stable neohookean, you should bias the lambda and mu:
  // Real lambda = lame[0] + 5.0 / 6.0 * lame[1], mu = 4.0 / 3.0 * lame[1];
  math::RealField1 weight = (lame.row(0) + 2 * lame.row(1)) * (dt * dt);
  math::RealSparseMatrix laplacian = LaplaceMatrixCompute<dim>{*(this->mesh_)}(weight);
  auto convect_diffuse = this->mass_matrix_original_ + laplacian;
  math::RealSparseMatrix full = math::kronecker_identity<dim>(convect_diffuse);
  this->mesh_->FilterMatrixFull(full);
  solver_->SetProblem(full).Compute();
}

template <int dim>
math::RealSparseMatrix Timestepper_QuasiNewton<dim>::GetLaplacianAsApproximation() const {
  const auto &lame = this->lame_;
  Real dt = this->dt_;
  math::RealField1 weight = (lame.row(0) + 2 * lame.row(1)) * (dt * dt);
  math::RealSparseMatrix laplace = LaplaceMatrixCompute<dim>{*(this->mesh_)}(weight);
  math::RealSparseMatrix full_laplacian = this->mass_matrix_original_ + laplace;
  this->mesh_->FilterMatrixDof(0, full_laplacian);
  return full_laplacian;
}

template <int dim>
void Timestepper_QuasiNewton<dim>::BeginSimulation(Real dt) {
  TimeStepperBase<dim>::BeginSimulation(dt);
  UpdateSolverLaplace();
  static_inverse_approximation_ = GetLaplacianAsApproximation();
  static_inverse_approximation_ = math::kronecker_identity<3>(static_inverse_approximation_);
}

template <int dim>
void Timestepper_QuasiNewton<dim>::BeginTimestep() {
  TimeStepperBase<dim>::BeginTimestep();
  if (strategy_ == LbfgsStrategy::kHard) {
    auto A = this->Hessian(this->du_inertia_);
    this->mesh_->FilterMatrixFull(A);
    solver_->SetProblem(std::move(A)).Compute();
  } else if (strategy_ == LbfgsStrategy::kReservedForExperimental) {
    AX_WARN("You are using the experimental mode!");
  }
}

class PreconditionerByBasis : public math::PreconditionerBase {
public:
  explicit PreconditionerByBasis(math::RealMatrixX const &basis,
                                 math::RealMatrixX const &inverse_under_basis)
      : basis_(basis), inverse_under_basis_(inverse_under_basis) {}

  math::RealMatrixX Solve(const math::RealMatrixX &b) override {
    math::RealMatrixX result = b;
    // project the b to the basis
    math::RealMatrixX b_under_basis = basis_.transpose() * b;
    // solve the reduced system
    math::RealMatrixX result_under_basis = inverse_under_basis_ * b_under_basis;
    // assemble the result
    result = basis_ * result_under_basis;
    return result + (b - basis_ * b_under_basis);
  }

  void AnalyzePattern() override {}

  void Factorize() override {}

  math::PreconditionerKind GetKind() const override { return math::PreconditionerKind::Identity; }

private:
  math::RealMatrixX basis_;
  math::RealMatrixX inverse_under_basis_;
};

template <int dim>
void Timestepper_QuasiNewton<dim>::SolveTimestep() {
  using namespace optim;
  Optimizer_Lbfgs &optimizer = optimizer_;
  optimizer.SetTolGrad(this->rel_tol_grad_);
  optimizer.SetTolVar(this->tol_var_);
  optimizer.SetMaxIter(this->max_iter_);

  if (strategy_ == LbfgsStrategy::kHard) {
    optimizer.SetApproxSolve(
        [&](Gradient const &g, Variable const &, Variable const &, Gradient const &) -> Variable {
          auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
          return approx.solution_;
        });
  } else if (strategy_ == LbfgsStrategy::kLaplacian) {
    optimizer.SetApproxSolve(
        [&](Gradient const &g, Variable const &, Variable const &, Gradient const &) -> Variable {
          auto start = utils::now();
          auto approx = solver_->Solve(g, g * this->dt_ * this->dt_);
          auto end = utils::now();
          AX_INFO("Solver Time Elapsed: {}", end - start);
          return approx.solution_;
        });
  } else if (strategy_ == LbfgsStrategy::kReservedForExperimental) {
    optimizer_.SetApproxSolve([&](Gradient const &gk, Variable const &du, Gradient const &sk,
                                  Gradient const &yk) -> Variable {
      AX_NOT_IMPLEMENTED();
      AX_UNREACHABLE();
    });
  }

  auto problem = this->AssembleProblem();
  OptResult result;
  try {
    result = optimizer.Optimize(problem, this->du_inertia_.reshaped());
  } catch (std::exception const &e) {
    AX_ERROR("Timestep solve failed: {}", e.what());
    return;
  }

  // SECT: Check the convergency result.
  if (!result.converged_) {
    AX_ERROR("Failed to converge, early stopped!");
  }

  AX_INFO("Converged: {}(v: {}, g: {}), Iterations={}",
          result.converged_var_ || result.converged_grad_, result.converged_var_,
          result.converged_grad_, result.n_iter_);
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template <int dim>
void Timestepper_QuasiNewton<dim>::SetOptions(const utils::Options &option) {
  utils::extract_enum(option, "lbfgs_strategy", strategy_);
  utils::extract_and_create<math::HostSparseSolverBase, math::HostSparseSolverKind>(
      option, "sparse_solver", solver_);
  extract_tunable(option, "sparse_solver_opt", solver_.get());
  /* SECT: Lbfgs Options */
  extract_tunable(option, "optimizer_opt", &optimizer_);
  /* SECT: More Options */
  TimeStepperBase<dim>::SetOptions(option);
}

template <int dim>
utils::Options Timestepper_QuasiNewton<dim>::GetOptions() const {
  auto option = TimeStepperBase<dim>::GetOptions();
  option["lbfgs_strategy"] = utils::reflect_name(strategy_).value();
  option["optimizer_opt"] = optimizer_.GetOptions();
  if (solver_) {
    option["sparse_solver"] = utils::reflect_name(solver_->GetKind()).value();
    option["sparse_solver_opt"] = solver_->GetOptions();
  }
  return option;
}

template class Timestepper_QuasiNewton<2>;
template class Timestepper_QuasiNewton<3>;

}  // namespace ax::fem
