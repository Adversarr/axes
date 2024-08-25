#include "ax/fem/timestepper/naive_optim.hpp"

#include <string>

#include "ax/core/excepts.hpp"
#include "ax/optim/optimizer_base.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/utils/opt.hpp"
#undef ERROR

namespace ax::fem {

template <int dim>
Timestepper_NaiveOptim<dim>::Timestepper_NaiveOptim(std::shared_ptr<TriMesh<dim>> mesh)
    : ax::fem::TimeStepperBase<dim>(mesh) {
  optimizer_ = std::make_unique<optim::Optimizer_Newton>();
}

template <int dim>
void Timestepper_NaiveOptim<dim>::SolveTimestep() {
  AX_THROW_IF_NULL(optimizer_, "Optimizer is not set.");
  optimizer_->SetTolVar(this->tol_var_);
  optimizer_->SetTolGrad(this->rel_tol_grad_);
  optimizer_->SetMaxIter(this->max_iter_);

  optim::OptProblem problem = this->AssembleProblem();
  optim::OptResult result;
  try {
    result = optimizer_->Optimize(problem, this->du_inertia_.reshaped());
  } catch (const std::exception& e) {
    AX_ERROR("Timestep solve failed: {}", e.what());
  }
  if (!(result.converged_grad_ || result.converged_var_)) {
    AX_ERROR("Failed to converge!");
  }

  AX_INFO("convergency: grad={}, var={}, iter={}", result.converged_grad_, result.converged_var_,
          result.n_iter_);

  // Set the final solution.
  this->du_ = result.x_opt_.reshaped(dim, this->mesh_->GetNumVertices());
}

template <int dim>
void Timestepper_NaiveOptim<dim>::SetOptions(const utils::Options& opt) {
  TimeStepperBase<dim>::SetOptions(opt);
  // if (auto it = opt.find("optimizer"); it != opt.end()) {
  //   auto const& o = it->value();
  //   if (!o.is_string()) {
  //     throw RuntimeError("Expect 'optimizer' to be a string.");
  //   }
  //   auto opt_kind = utils::reflect_enum<optim::OptimizerKind>(o.as_string().c_str());
  //   if (!opt_kind) {
  //     throw RuntimeError("Failed to reflect optimizer: " + std::string(o.as_string()));
  //   }
  //   optimizer_ = optim::OptimizerBase::Create(opt_kind.value());
  //   AX_THROW_IF_NULL(optimizer_, "Failed to create optimizer: " + std::string(o.as_string()));
  // }
  // auto [has_opt, optimizer] = utils::extract_enum<optim::OptimizerKind>(opt, "optimizer");
  // if (has_opt) {
  //   AX_THROW_IF_NULL(optimizer, "Failed to reflect optimizer: "
  //                               + std::string(utils::extract_string(opt.at("optimizer"))));
  //   optimizer_ = optim::OptimizerBase::Create(optimizer.value());
  //   AX_THROW_IF_NULL(optimizer_, "Failed to create optimizer: "
  //                                + std::string(utils::extract_string(opt.at("optimizer"))));
  // }

  utils::extract_and_create<optim::OptimizerBase, optim::OptimizerKind>(opt, "optimizer",
                                                                        optimizer_);
  if (optimizer_) {
    utils::extract_tunable(opt, "optimizer_opt", optimizer_.get());
  }
}

template <int dim>
utils::Options Timestepper_NaiveOptim<dim>::GetOptions() const {
  auto opt = TimeStepperBase<dim>::GetOptions();
  opt["optimizer"] = utils::reflect_name(optimizer_->GetKind()).value();
  opt["optimizer_opt"] = optimizer_->GetOptions();
  return opt;
}

template <int dim>
optim::OptimizerBase* Timestepper_NaiveOptim<dim>::GetOptimizer() const {
  return optimizer_.get();
}

template class Timestepper_NaiveOptim<2>;
template class Timestepper_NaiveOptim<3>;

}  // namespace ax::fem
