#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/optim/optimizers/newton.hpp"
#undef ERROR

namespace ax::fem {

template<idx dim>
void Timestepper_NaiveOptim<dim>::SolveTimestep() {
  optim::Newton optimizer;
  optim::OptProblem problem = this->AssembleProblem();
  optimizer.SetTolGrad(this->rel_tol_grad_);
  optimizer.SetMaxIter(this->max_iter_);
  optim::OptResult result;
  try {
    result = optimizer.Optimize(problem, this->du_inertia_.reshaped());
  } catch (const std::exception& e) {
    AX_LOG(ERROR) << "Timestep solve failed: " << e.what();
  }
  if (!(result.converged_grad_ || result.converged_var_)) {
    AX_LOG(ERROR) << "Optimizer failed to converge!";
  }

  AX_LOG(INFO) << "#Iter: " << result.n_iter_ << " iterations.";

  // Set the final solution.
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template class Timestepper_NaiveOptim<2>;
template class Timestepper_NaiveOptim<3>;

}  // namespace ax::fem
