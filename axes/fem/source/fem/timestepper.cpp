#include "ax/fem/timestepper.hpp"

#ifdef AX_HAS_CUDA
#  include "ax/fem/elasticity_gpu.cuh"
#endif

#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/utils/iota.hpp"

namespace ax::fem {

template <idx dim> TimeStepperBase<dim>::TimeStepperBase() {
  integration_scheme_ = TimestepSchemeBase<dim>::Create(TimestepSchemeKind::kBackwardEuler);
  u_lame_ = elasticity::compute_lame(youngs_, poisson_ratio_);
}

template <idx dim> TimeStepperBase<dim>::TimeStepperBase(SPtr<TriMesh<dim>> mesh)
    : TimeStepperBase() {
  mesh_ = std::move(mesh);
}

template <idx dim> Status TimeStepperBase<dim>::Initialize() {
  AX_THROW_IF_NULLPTR(mesh_);
  idx n_vert = mesh_->GetNumVertices();
  u_.setZero(dim, n_vert);
  u_back_ = u_;
  du_ = u_;
  du_inertia_ = u_;
  velocity_.setZero(dim, n_vert);
  velocity_back_ = velocity_;
  ext_accel_.setZero(dim, n_vert);

  if (mass_matrix_.size() == 0) {
    AX_LOG(WARNING) << "Mass matrix is not set, use density uniformly = " << density_;
    SetDensity(density_);
  }
  AX_RETURN_OK();
}

template <idx dim> void TimeStepperBase<dim>::SetOptions(utils::Opt const& opt) {
  AX_SYNC_OPT_IF(opt, real, rel_tol_grad) {
    AX_THROW_IF_LT(rel_tol_grad_, 1e-6, "The relative tol grad is too small");
  }
  AX_SYNC_OPT_IF(opt, real, tol_var) { AX_LOG(INFO) << "Tol Variance: " << tol_var_; }
  AX_SYNC_OPT_IF(opt, idx, max_iter) { AX_LOG(INFO) << "Max Iteration: " << max_iter_; }
  AX_SYNC_OPT(opt, bool, record_trajectory);
  if (auto it = opt.find("conv_norm"); it != opt.end()) {
    auto conv_norm = utils::reflect_enum<TimestepConvergeNormKind>(it->value().as_string().c_str());
    AX_THROW_IF_NULL(conv_norm);
    converge_kind_ = *conv_norm;
  }
  if (auto it = opt.find("integration_scheme"); it != opt.end()) {
    auto scheme_kind = utils::reflect_enum<TimestepSchemeKind>(it->value().as_string().c_str());
    AX_THROW_IF_NULL(scheme_kind);
    integration_scheme_ = TimestepSchemeBase<dim>::Create(*scheme_kind);
    AX_THROW_IF_NULL(integration_scheme_);
  }

  std::string ename, device;
  if (auto it = opt.find("elasticity"); it != opt.end()) {
    ename = utils::extract_string(it->value());
  }
  if (auto it = opt.find("device"); it != opt.end()) {
    device = utils::extract_string(it->value());
  }
  if (!ename.empty() || device.empty()) {
    SetupElasticity(ename.empty() ? elasticity_name_ : ename, device.empty() ? device_ : device);
  }

  AX_SYNC_OPT_IF(opt, real, youngs) {
    AX_THROW_IF_LT(youngs_, 0, "Young's modulus should be positive.");
  }
  AX_SYNC_OPT_IF(opt, real, poisson_ratio) {
    AX_THROW_IF_FALSE(0 < poisson_ratio_ && poisson_ratio_ < 0.5,
                      "Poisson ratio should be in (0, 0.5).");
    if (poisson_ratio_ > 0.49) {
      AX_LOG_FIRST_N(WARNING, 1)
          << "Poisson ratio is close to 0.5, which may cause numerical instability.";
    }
  }
  u_lame_ = elasticity::compute_lame(youngs_, poisson_ratio_);

  AX_SYNC_OPT_IF(opt, real, density) { SetDensity(density_); }
  utils::Tunable::SetOptions(opt);
}

template <idx dim>
void TimeStepperBase<dim>::SetupElasticity(std::string name, std::string device) {
  if (!mesh_) {
    throw LogicError("Cannot setup elasticity when mesh is not set.");
  }
#ifdef AX_HAS_CUDA
#  define DeclareElasticity(en, tpl)                     \
    do {                                                 \
      if (name == en) {                                  \
        elasticity_name_ = en;                           \
        if (device == "gpu") {                           \
          device_ = "gpu";                               \
          SetupElasticity<tpl, ElasticityCompute_GPU>(); \
        } else {                                         \
          device = "cpu";                                \
          SetupElasticity<tpl, ElasticityCompute_CPU>(); \
        }                                                \
        return;                                          \
      }                                                  \
    } while (0)
#else
#  define DeclareElasticity(en, tpl)                                      \
    do {                                                                  \
      if (name == en) {                                                   \
        elasticity_name_ = en;                                            \
        device_ = "cpu";                                                  \
        if (device == "gpu") {                                            \
          throw InvalidArgument("'gpu' is not supported in this build."); \
        } else {                                                          \
          SetupElasticity<tpl, ElasticityCompute_CPU>();                  \
        }                                                                 \
        return;                                                           \
      }                                                                   \
    } while (0)
#endif
  DeclareElasticity("linear", elasticity::Linear);
  DeclareElasticity("isotropic_arap", elasticity::IsotropicARAP);
  DeclareElasticity("neohookean_bw", elasticity::NeoHookeanBW);
  DeclareElasticity("stable_neohookean", elasticity::StableNeoHookean);
  DeclareElasticity("stvk", elasticity::StVK);
  throw InvalidArgument("Elasticity model" + name + " not found.");
}

template <idx dim> utils::Opt TimeStepperBase<dim>::GetOptions() const {
  utils::Opt opt = utils::Tunable::GetOptions();
  opt["rel_tol_grad"] = rel_tol_grad_;
  opt["tol_var"] = tol_var_;
  opt["max_iter"] = max_iter_;
  opt["record_trajectory"] = record_trajectory_;
  opt["conv_norm"] = utils::reflect_name(converge_kind_).value();
  opt["integration_scheme"] = utils::reflect_name(integration_scheme_->GetKind()).value();
  opt["elasticity"] = elasticity_name_;
  opt["device"] = device_;
  opt["youngs"] = youngs_;
  opt["poisson_ratio"] = poisson_ratio_;
  opt["density"] = density_;
  return opt;
}

template <idx dim> void TimeStepperBase<dim>::SetDensity(real density) {
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_original_ = mmc(density);
  mass_matrix_ = math::kronecker_identity<dim>(mass_matrix_original_);
}

template <idx dim> void TimeStepperBase<dim>::SetDensity(math::field1r const& density) {
  auto mmc = MassMatrixCompute<dim>(*mesh_);
  mass_matrix_original_ = mmc(density);
  mass_matrix_ = math::kronecker_identity<dim>(mass_matrix_original_);
}

template <idx dim>
math::sp_matxxr TimeStepperBase<dim>::GetStiffnessMatrix(math::fieldr<dim> const& x,
                                                         bool project) const {
  auto lame = u_lame_;
  elasticity_->Update(x, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(project);
  elasticity_->GatherHessianToVertices();
  return elasticity_->GetHessianOnVertices();
}

template <idx dim>
math::fieldr<dim> TimeStepperBase<dim>::GetElasticForce(math::fieldr<dim> const& x) const {
  auto lame = u_lame_;
  elasticity_->Update(x, ElasticityUpdateLevel::kStress);
  elasticity_->UpdateStress();
  elasticity_->GatherStressToVertices();
  return elasticity_->GetStressOnVertices();
}

template <idx dim> void TimeStepperBase<dim>::BeginSimulation(real) {
  has_time_step_begin_ = false;
  u_lame_ = elasticity::compute_lame(youngs_, poisson_ratio_);
  AX_THROW_IF_NULLPTR(mesh_);
  AX_THROW_IF_NULLPTR(elasticity_);
  AX_THROW_IF_NULLPTR(integration_scheme_);
  elasticity_->RecomputeRestPose();
  return;
}

template <idx dim> void TimeStepperBase<dim>::BeginTimestep(real dt) {
  if (has_time_step_begin_) {
    throw LogicError("Timestep has already begun. Call EndTimestep before starting a new one.");
  }
  idx n_vert = mesh_->GetNumVertices();

  integration_scheme_->SetDeltaT(dt);
  du_inertia_ = integration_scheme_->Precomputed(mass_matrix_original_, u_, u_back_, velocity_,
                                                 velocity_back_, ext_accel_);
  u_inertia_ = u_ + du_inertia_;
  mesh_->FilterField(du_inertia_, true);
  mesh_->FilterField(du_, true);
  auto V = GetPosition();
  // For the Dirichlet BCs in inertia term, directly set the displacement.
  for (auto [i, d] : utils::multi_iota(n_vert, dim)) {
    if (mesh_->IsDirichletBoundary(i, d)) {
      du_inertia_(d, i) = mesh_->GetBoundaryValue(i, d) - V(d, i);
    }
  }
  du_ = du_inertia_;

  // TODO: Lame parameters should be set uniformly or per element.
  elasticity_->SetLame(u_lame_);

  // convergency test parameters
  math::fieldr<dim> body_force = (math::fieldr<dim>::Ones(dim, n_vert) * mass_matrix_original_);
  abs_tol_grad_ = dt * dt * ResidualNorm(body_force) + math::epsilon<real>;
  AX_LOG(INFO) << "Absolute tolerance for gradient: " << abs_tol_grad_;

  has_time_step_begin_ = true;
  return;
}

template <idx dim> void TimeStepperBase<dim>::EndTimestep() { EndTimestep(du_); }

template <idx dim> void TimeStepperBase<dim>::EndTimestep(math::fieldr<dim> const& du) {
  if (!has_time_step_begin_) {
    throw LogicError("Timestep has not begun. Call BeginTimestep before ending one.");
  }
  velocity_back_ = integration_scheme_->NewVelocity(u_, u_back_, velocity_, velocity_back_, du);
  std::swap(u_back_, u_);
  std::swap(velocity_back_, velocity_);
  u_.noalias() = du + u_back_;
  has_time_step_begin_ = false;
}

template <idx dim> void TimeStepperBase<dim>::SolveTimestep() {
  // Do nothing, do not throw anything, for testing.
  AX_LOG(ERROR) << "SolveTimestep is not implemented!";
  du_.setZero();
}

template <idx dim> real TimeStepperBase<dim>::Energy(math::fieldr<dim> const& u) const {
  math::fieldr<dim> x_new = u + mesh_->GetVertices();
  elasticity_->Update(x_new, ElasticityUpdateLevel::kEnergy);
  elasticity_->UpdateEnergy();
  real stiffness = elasticity_->GetEnergyOnElements().sum();
  math::fieldr<dim> duu = u - u_inertia_;
  math::fieldr<dim> Mdx = 0.5 * duu * mass_matrix_original_;
  real inertia = (duu.array() * Mdx.array()).sum();
  return integration_scheme_->ComposeEnergy(inertia, stiffness);
}

template <idx dim>
math::fieldr<dim> TimeStepperBase<dim>::Gradient(math::fieldr<dim> const& u_cur) const {
  math::fieldr<dim> x_new = u_cur + mesh_->GetVertices();
  elasticity_->Update(x_new, ElasticityUpdateLevel::kStress);
  elasticity_->UpdateStress();
  elasticity_->GatherStressToVertices();
  math::fieldr<dim> neg_force = elasticity_->GetStressOnVertices();
  math::fieldr<dim> grad
      = integration_scheme_->ComposeGradient(mass_matrix_original_, u_cur, neg_force, u_inertia_);
  mesh_->FilterField(grad, true);
  return grad;
}

template <idx dim> math::vecxr TimeStepperBase<dim>::GradientFlat(math::vecxr const& u_cur) const {
  idx n_vert = mesh_->GetNumVertices();
  return Gradient(u_cur.reshaped(dim, n_vert)).reshaped();
}

template <idx dim> math::sp_matxxr TimeStepperBase<dim>::Hessian(math::fieldr<dim> const& u) const {
  math::fieldr<dim> x_new = u + mesh_->GetVertices();
  elasticity_->Update(x_new, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(true);
  elasticity_->GatherHessianToVertices();
  auto stiffness = elasticity_->GetHessianOnVertices();
  auto H = integration_scheme_->ComposeHessian(mass_matrix_, stiffness);
  mesh_->FilterMatrixFull(H);
  return H;
}

template <idx dim> optim::OptProblem TimeStepperBase<dim>::AssembleProblem() const {
  optim::OptProblem problem;
  auto n_vert = mesh_->GetNumVertices();
  problem
      .SetEnergy([this, n_vert](math::vecxr const& du) -> real {
        return Energy(du.reshaped(dim, n_vert) + u_);
      })
      .SetGrad([this, n_vert](math::vecxr const& du) -> math::vecxr {
        return GradientFlat(du + u_.reshaped());
      })
      .SetSparseHessian([this, n_vert](math::vecxr const& du) -> math::sp_matxxr {
        return Hessian(du.reshaped(dim, n_vert) + u_);
      })
      .SetConvergeGrad([this, n_vert](const math::vecxr&, const math::vecxr& grad) -> real {
        return ResidualNorm(grad.reshaped(dim, n_vert)) / abs_tol_grad_;
      })
      .SetConvergeVar([this](const math::vecxr& du_back, const math::vecxr& du) -> real {
        return ResidualNorm(du_back - du);
      });

  if (record_trajectory_) {
    problem.SetVerbose([this](idx ith, const math::vecxr& x, const real energy) {
      auto* self = const_cast<TimeStepperBase<dim>*>(this);
      if (ith == 0) {
        self->last_trajectory_.clear();
        self->last_energy_.clear();
      }
      self->last_trajectory_.push_back(x.reshaped(dim, mesh_->GetNumVertices()));
      self->last_energy_.push_back(energy);
    });
  }
  return problem;
}

template <idx dim> real TimeStepperBase<dim>::ResidualNorm(math::fieldr<dim> const& grad) const {
  switch (converge_kind_) {
    case TimestepConvergeNormKind::kL1:
      return L1Residual(grad);
    case TimestepConvergeNormKind::kLinf:
      return LinfResidual(grad);
    case TimestepConvergeNormKind::kL2:
    default:
      return L2Residual(grad);
  }
}

template <idx dim> real TimeStepperBase<dim>::L2Residual(math::fieldr<dim> const& grad) const {
  return math::norm(grad, math::l2);
}

template <idx dim> real TimeStepperBase<dim>::L1Residual(math::fieldr<dim> const& grad) const {
  return math::norm(grad, math::l1);
}

template <idx dim> real TimeStepperBase<dim>::LinfResidual(math::fieldr<dim> const& grad) const {
  return math::norm(grad, math::linf);
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}  // namespace ax::fem
