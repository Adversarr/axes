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

template <idx dim> TimeStepperBase<dim>::TimeStepperBase(std::shared_ptr<TriMesh<dim>> mesh) {
  u_lame_ = elasticity::compute_lame(youngs_, poisson_ratio_);
  mesh_ = mesh;
  mesh_->SetNumDofPerVertex(dim);
  integration_scheme_ = TimestepSchemeBase<dim>::Create(TimestepSchemeKind::kBackwardEuler);
  has_initialized_ = false;
  has_time_step_begin_ = false;
}

template <idx dim> Status TimeStepperBase<dim>::Initialize() {
  AX_THROW_IF_TRUE(has_initialized_, "TimeStepper has already been initialized.");
  AX_THROW_IF_NULLPTR(mesh_, "Mesh is not set.");
  if (!elasticity_) {
    AX_LOG(WARNING) << "Elasticity not set, use: " << elasticity_name_ << " device: " << device_;
    SetupElasticity(elasticity_name_, device_);
  }

  idx n_vert = mesh_->GetNumVertices();
  // state variables
  AX_LOG(INFO) << "Initialize TimeStepper with " << n_vert << " vertices.";
  u_.setZero(dim, n_vert);
  u_back_.setZero(dim, n_vert);
  velocity_.setZero(dim, n_vert);
  velocity_back_.setZero(dim, n_vert);
  ext_accel_.setZero(dim, n_vert);

  // solve results
  du_.setZero(dim, n_vert);
  u_inertia_.setZero(dim, n_vert);
  du_inertia_.setZero(dim, n_vert);

  if (mass_matrix_.size() == 0) {
    AX_LOG(INFO) << "Mass matrix is not set, use density uniformly = " << density_;
    SetDensity(density_);
  }
  has_initialized_ = true;
  u_lame_ = elasticity::compute_lame(youngs_, poisson_ratio_);
  AX_RETURN_OK();
}

template <idx dim> void TimeStepperBase<dim>::SetOptions(utils::Options const& opt) {
  AX_THROW_IF_TRUE(has_initialized_, "Cannot set options after initialization.");

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
  DeclareElasticity("stvk", elasticity::StVK);
  DeclareElasticity("stable_neohookean", elasticity::StableNeoHookean);
  throw InvalidArgument("Elasticity model" + name + " not found.");
}

template <idx dim> utils::Options TimeStepperBase<dim>::GetOptions() const {
  utils::Options opt = utils::Tunable::GetOptions();
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

template <idx dim> math::fieldr<dim> TimeStepperBase<dim>::GetPosition() const {
  AX_THROW_IF_TRUE(u_.cols() != mesh_->GetNumVertices());
  math::fieldr<dim> x = u_ + mesh_->GetVertices();
  return x;
}

template <idx dim> math::fieldr<dim> TimeStepperBase<dim>::GetLastPosition() const {
  return u_back_ + mesh_->GetVertices();
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

template <idx dim> math::spmatr TimeStepperBase<dim>::GetStiffnessMatrix(math::fieldr<dim> const& u,
                                                                         bool project) const {
  elasticity_->Update(u, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(project);
  elasticity_->GatherHessianToVertices();
  return elasticity_->GetHessianOnVertices();
}

template <idx dim>
math::fieldr<dim> TimeStepperBase<dim>::GetElasticForce(math::fieldr<dim> const& u) const {
  elasticity_->Update(u, ElasticityUpdateLevel::kStress);
  elasticity_->UpdateStress();
  elasticity_->GatherStressToVertices();
  return elasticity_->GetStressOnVertices();
}

template <idx dim> void TimeStepperBase<dim>::BeginSimulation(real dt) {
  AX_THROW_IF_NULLPTR(mesh_);
  AX_THROW_IF_NULLPTR(elasticity_);
  AX_THROW_IF_NULLPTR(integration_scheme_);
  AX_THROW_IF_TRUE(has_simulation_begun_, "Simulation has already begun.");

  this->dt_ = dt;
  integration_scheme_->SetDeltaT(dt);
  elasticity_->SetLame(u_lame_);
  elasticity_->RecomputeRestPose();
  idx n_vert = mesh_->GetNumVertices();
  math::fieldr<dim> body_force = (math::fieldr<dim>::Ones(dim, n_vert) * mass_matrix_original_);
  abs_tol_grad_ = dt_ * dt_ * ResidualNorm(body_force) + math::epsilon<real>;
  AX_LOG(INFO) << "Absolute tolerance for gradient: " << abs_tol_grad_;
  has_time_step_begin_ = false;
  has_simulation_begun_ = true;
  return;
}

template <idx dim> void TimeStepperBase<dim>::BeginTimestep() {
  AX_THROW_IF_FALSE(has_simulation_begun_, "Simulation has not begun. Call BeginSimulation first.");
  if (has_time_step_begin_) {
    throw LogicError("Timestep has already begun. Call EndTimestep before starting a new one.");
  }
  RecomputeInitialGuess(u_, u_back_, velocity_, velocity_back_, ext_accel_);
  // TODO: Lame parameters should be set uniformly or per element.
  // convergency test parameters
  has_time_step_begin_ = true;
  return;
}

template <idx dim> void TimeStepperBase<dim>::EndTimestep() { EndTimestep(du_); }

template <idx dim> void TimeStepperBase<dim>::EndTimestep(math::fieldr<dim> const& du) {
  if (!has_time_step_begin_) {
    throw LogicError("Timestep has not begun. Call BeginTimestep before ending one.");
  }

  if (du.cols() != u_.cols()) {
    throw InvalidArgument("Invalid shape of du: " + std::to_string(du.cols())
                          + " != " + std::to_string(u_.cols()));
  }
  velocity_back_ = integration_scheme_->NewVelocity(u_, u_back_, velocity_, velocity_back_, du);
  velocity_back_.swap(velocity_);
  u_back_ = u_;  // last displacement
  u_ = du + u_back_;
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
  math::fieldr<dim> du = u - u_inertia_;
  math::fieldr<dim> Mdu = 0.5 * du * mass_matrix_original_;
  real inertia = (du.array() * Mdu.array()).sum();
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
  auto g = Gradient(u_cur.reshaped(dim, n_vert)).reshaped();
  return g;
}

template <idx dim>
math::spmatr TimeStepperBase<dim>::Hessian(math::fieldr<dim> const& u, bool project) const {
  math::fieldr<dim> x_new = u + mesh_->GetVertices();
  // math::fieldr<dim> du = u - u_;
  elasticity_->Update(x_new, ElasticityUpdateLevel::kHessian);
  elasticity_->UpdateHessian(project);
  elasticity_->GatherHessianToVertices();
  auto const& stiffness = elasticity_->GetHessianOnVertices();
  auto H = integration_scheme_->ComposeHessian(mass_matrix_, stiffness);
  mesh_->FilterMatrixFull(H);
  return H;
}

template <idx dim> optim::OptProblem TimeStepperBase<dim>::AssembleProblem() {
  optim::OptProblem problem;
  auto n_vert = mesh_->GetNumVertices();
  problem
      .SetEnergy([&, n_vert](math::vecxr const& du) -> real {
        return Energy(du.reshaped(dim, n_vert) + u_);
      })
      .SetGrad([&](math::vecxr const& du) -> math::vecxr {
        math::vecxr g = GradientFlat(du + u_.reshaped());
        return g;
      })
      .SetSparseHessian([&, n_vert](math::vecxr const& du) -> math::spmatr {
        auto H = Hessian(du.reshaped(dim, n_vert) + u_, true);
        return H;
      })
      .SetConvergeGrad([this, n_vert](const math::vecxr&, const math::vecxr& grad) -> real {
        return ResidualNorm(grad.reshaped(dim, n_vert)) / abs_tol_grad_;
      })
      .SetConvergeVar([this, n_vert](const math::vecxr& du_back, const math::vecxr& du) -> real {
        math::fieldr<dim> ddu_flat = (du_back - du).reshaped(dim, n_vert);
        return ResidualNorm(ddu_flat);
      });

  last_trajectory_.clear();
  last_energy_.clear();
  last_iterations_ = 0;
  problem.SetVerbose([this](idx ith, const math::vecxr& x, const real energy) {
    x.reshaped<Eigen::AutoOrder>(dim, mesh_->GetNumVertices());
    auto du =  math::reshape<dim>(x, mesh_->GetNumVertices());
    last_trajectory_.push_back(u_ + du);
    last_energy_.push_back(energy);
    last_iterations_ = ith;
  });
  return problem;
}

template <idx dim> void TimeStepperBase<dim>::RecomputeInitialGuess(
    math::fieldr<dim> const& u, math::fieldr<dim> const& u_back, math::fieldr<dim> const& velocity,
    math::fieldr<dim> const& velocity_back, math::fieldr<dim> const& ext_accel) {
  idx n_vert = mesh_->GetNumVertices();
  du_inertia_ = integration_scheme_->Precomputed(mass_matrix_original_, u, u_back, velocity,
                                                 velocity_back, ext_accel);
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

template <idx dim> template <template <idx> class ElasticModelTemplate,
                             template <idx, template <idx> class> class Compute>
void TimeStepperBase<dim>::SetupElasticity() {
  elasticity_ = std::make_unique<Compute<dim, ElasticModelTemplate>>(mesh_);
}

template class TimeStepperBase<2>;
template class TimeStepperBase<3>;

}  // namespace ax::fem
