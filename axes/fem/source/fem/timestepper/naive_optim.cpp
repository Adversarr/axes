#include "ax/fem/timestepper/naive_optim.hpp"

#include "ax/optim/optimizers/newton.hpp"
#undef ERROR

namespace ax::fem {

template <idx dim> Timestepper_NaiveOptim<dim>::Timestepper_NaiveOptim() {
  optimizer_ = std::make_unique<optim::Newton>();
}

template <idx dim> Timestepper_NaiveOptim<dim>::Timestepper_NaiveOptim(SPtr<TriMesh<dim>> mesh)
    : Timestepper_NaiveOptim() {
  this->mesh_ = mesh;
}

template <idx dim> void Timestepper_NaiveOptim<dim>::SolveTimestep() {
  optimizer_->SetTolVar(this->tol_var_);
  optimizer_->SetTolGrad(this->rel_tol_grad_);
  optimizer_->SetMaxIter(this->max_iter_);

  optim::OptProblem problem = this->AssembleProblem();
  optim::OptResult result;
  try {
    result = optimizer_->Optimize(problem, this->du_inertia_.reshaped());
  } catch (const std::exception& e) {
    AX_LOG(ERROR) << "Timestep solve failed: " << e.what();
  }
  if (!(result.converged_grad_ || result.converged_var_)) {
    AX_LOG(ERROR) << "optimizer_->failed to converge!";
  }

  AX_LOG(INFO) << "#Iter: " << result.n_iter_ << " iterations.";

  // Set the final solution.
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template <idx dim> void Timestepper_NaiveOptim<dim>::SetOptions(const utils::Opt& opt) {
  TimeStepperBase<dim>::SetOptions(opt);
  if (auto it = opt.find("optimizer"); it != opt.end()) {
    auto const& o = it->value();
    if (!o.is_string()) {
      throw RuntimeError("Expect 'optimizer' to be a string.");
    }
    auto opt_kind = utils::reflect_enum<optim::OptimizerKind>(o.as_string().c_str());
    if (!opt_kind) {
      throw RuntimeError("Failed to reflect optimizer: " + std::string(o.as_string()));
    }
    optimizer_ = optim::OptimizerBase::Create(opt_kind.value());
    AX_THROW_IF_NULL(optimizer_, "Failed to create optimizer: " + std::string(o.as_string()));
  }
  if (optimizer_) {
    utils::extract_tunable(opt, "optimizer_options", optimizer_.get());
  }
}

template <idx dim> utils::Opt Timestepper_NaiveOptim<dim>::GetOptions() const {
  auto opt = TimeStepperBase<dim>::GetOptions();
  opt["optimizer"] = utils::reflect_name(optimizer_->GetKind()).value();
  opt["optimizer_options"] = optimizer_->GetOptions();
  return opt;
}

template class Timestepper_NaiveOptim<2>;
template class Timestepper_NaiveOptim<3>;

}  // namespace ax::fem
