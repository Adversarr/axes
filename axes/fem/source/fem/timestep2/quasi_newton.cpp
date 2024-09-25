#include "ax/fem/timestep2/quasi_newton.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/fem/terms/laplace.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/block_jacobi.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"
#include "ax/math/sparse_matrix/linsys/solver/downcast.hpp"
#include "ax/optim2/optimizer/lbfgs.hpp"
#include "ax/utils/time.hpp"

namespace ax::fem {

TimeStep_QuasiNewton::TimeStep_QuasiNewton(shared_not_null<Mesh> mesh) : TimeStepBase(mesh) {
  // Create the approximate problem.
  auto state = problem_.GetState();
  approximate_problem_ = std::make_unique<Problem>(state, mesh_);
  approximate_problem_->AddTerm("mass", std::make_unique<MassTerm>(state, mesh_));
  auto& laplace
      = approximate_problem_->AddTerm("laplace", std::make_unique<LaplaceTerm>(state, mesh_));
  laplace.scale_ = dt_ * dt_;
}

TimeStep_QuasiNewton::~TimeStep_QuasiNewton() = default;

void TimeStep_QuasiNewton::Compute() {
  TimeStepBase::Compute();
  approximate_problem_->MarkDirty();
  approximate_problem_->InitializeHessianFillIn();
  approximate_problem_->UpdateHessian();
  auto hes = approximate_problem_->GetHessian();
  solver_ = std::make_unique<math::GeneralSparseSolver_ConjugateGradient>();  // hard coded.
  // solver_ = std::make_unique<math::GeneralSparseSolver_Downcast>();  // hard coded.
  solver_->preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_BlockJacobi>();
  solver_->SetProblem(hes);
  solver_->Compute();
  solver_->max_iteration_ = 30;  // do 10 step of pcg is enough for LBFGS preconditioning
}

void TimeStep_QuasiNewton::SolveStep() {
  // Implement the quasi-newton (Liu 17)
  optim2::Optimizer_LBFGS optimizer;
  auto start = utils::now();
  optimizer.SetProblem(PrepareVariationalProblem());
  // Solve the problem.
  MarkCurrentSolutionUpdated();  // start with a dirty.

  auto approximator = [&](RealBufferView r, ConstRealBufferView /* s */,
                          ConstRealBufferView /* y */, bool /* is_sy_valid */) {
    // solve the linear system.
    prune_dirichlet_bc_.PruneGradient(r);
    temp_ = ensure_buffer<Real>(temp_, r.Device(), r.Shape());
    copy(temp_->View(), r);
    auto result = solver_->Solve(temp_->View(), r);
    AX_UNUSED(result);
    prune_dirichlet_bc_.PruneGradient(r);
  };
  optimizer.tol_grad_ = tol_abs_grad_;
  optimizer.precond_ = approximator;
  optimizer.history_size_ = 5;
  auto result = optimizer.Optimize({});

  auto end = utils::now();
  AX_INFO("Optimal energy: {} |g|={} conv_grad={} iter={} time={}", result.f_opt_,
          math::buffer_blas::norm(problem_.GetGradient()), result.converged_grad_, result.n_iter_,
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
}

void TimeStep_QuasiNewton::SetDensity(ConstRealBufferView density) {
  TimeStepBase::SetDensity(density);

  // Update the density in the problem.
  auto* mass = static_cast<MassTerm*>(approximate_problem_->GetTerm("mass").term_.get());
  mass->SetDensity(density);
}

void TimeStep_QuasiNewton::SetLame(ConstRealBufferView lame) {
  TimeStepBase::SetLame(lame);

  // I appologize for the following code, but it is the simplest way to update the Lame parameters.
  auto temp_buf = create_buffer<Real>(BufferDevice::Host, lame.Shape());
  auto temp_view = temp_buf->View();
  copy(temp_view, lame);
  for (size_t i = 0; i < lame.Shape().Y(); ++i) {
    temp_buf->Data()[i] = temp_view(0, i) + 2 * temp_view(1, i);
  }

  // Update the Lame parameters in the problem.
  auto* laplace = static_cast<LaplaceTerm*>(approximate_problem_->GetTerm("laplace").term_.get());
  laplace->SetDiffusivity(view_from_raw_buffer(temp_buf->Data(), {lame.Shape().Y()}));
}

void TimeStep_QuasiNewton::SetTimeStep(Real dt) {
  TimeStepBase::SetTimeStep(dt);

  // Update the timestep in the problem.
  auto& laplace = approximate_problem_->GetTerm("laplace");
  laplace.scale_ = dt * dt;
}

}  // namespace ax::fem