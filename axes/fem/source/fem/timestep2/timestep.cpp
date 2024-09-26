#include "ax/fem/timestep2/timestep.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/fem/elasticity/base.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/optimizer/newton.hpp"
#include "ax/utils/time.hpp"

namespace ax::fem {

static std::unique_ptr<State> make_fem_state(std::shared_ptr<Mesh> mesh) {
  size_t n_dof_per_vertex = mesh->GetNumDOFPerVertex();
  size_t n_vert = mesh->GetNumVertices();
  auto device = mesh->Device();
  return std::make_unique<State>(n_dof_per_vertex, n_vert, device);
}

TimeStepBase::TimeStepBase(shared_not_null<Mesh> mesh)
    : problem_(make_fem_state(mesh), mesh), mesh_(mesh), prune_dirichlet_bc_(problem_.GetState()) {
  // displacement
  u_ = problem_.GetState()->GetVariables();
  u_back_ = u_->Clone();

  // velocity.
  velocity_ = u_->Clone();
  ext_accel_ = u_->Clone();
  temp_ = u_->Clone();

  // problem terms
  auto state = problem_.GetState();
  auto& mass = problem_.AddTerm("inertia", std::make_unique<MassTerm>(state, mesh));
  cache_inertia_ = static_cast<MassTerm*>(mass.term_.get());
  auto& elast = problem_.AddTerm("elasticity", std::make_unique<ElasticityTerm>(state, mesh));
  cache_elasticity_ = static_cast<ElasticityTerm*>(elast.term_.get());

  UpdatePruneDirichletBc();
  problem_.InitializeHessianFillIn();
}

const Problem& TimeStepBase::GetProblem() const {
  return problem_;
}

Problem& TimeStepBase::GetProblem() {
  return problem_;
}

not_null<MassTerm*> TimeStepBase::GetInertia() {
  return cache_inertia_;
}

not_null<ElasticityTerm*> TimeStepBase::GetElasticity() {
  return cache_elasticity_;
}

void TimeStepBase::SetElasticity(ElasticityKind kind) {
  cache_elasticity_->SetKind(kind);
  AX_INFO("Set elasticity kind to {}", utils::reflect_name(kind).value_or("???"));
}

void TimeStepBase::SetLame(ConstRealBufferView lame) {
  cache_elasticity_->SetLame(lame);
  is_set_lame_called_ = true;
}

void TimeStepBase::SetLame(const math::RealVector2& lame) {
  size_t num_elements = mesh_->GetNumElements();
  math::RealField2 lame_field(2, static_cast<Index>(num_elements));
  lame_field.colwise() = lame;
  SetLame(view_from_matrix(lame_field));
}

void TimeStepBase::SetDensity(ConstRealBufferView density) {
  cache_inertia_->SetDensity(density);
  is_set_density_called_ = true;
}

void TimeStepBase::SetDensity(Real density) {
  size_t num_elements = mesh_->GetNumElements();
  math::RealVectorX density_field(static_cast<Index>(num_elements));
  density_field.fill(density);
  SetDensity(flatten(view_from_matrix(density_field)));
}

void TimeStepBase::SetExternalAcceleration(ConstRealBufferView ext_accel) {
  copy(ext_accel_->View(), ext_accel);
}

void TimeStepBase::SetExternalAcceleration(const math::RealVector2& ext_accel) {
  size_t num_elements = mesh_->GetNumElements();
  size_t n_dof_per_vertex = mesh_->GetNumDOFPerVertex();
  AX_THROW_IF_NE(n_dof_per_vertex, 2, "This overload only works for 2D mesh.");

  math::RealField2 ext_accel_field(2, static_cast<Index>(num_elements));
  ext_accel_field.colwise() = ext_accel;
  SetExternalAcceleration(flatten(view_from_matrix(ext_accel_field)));
}

void TimeStepBase::SetExternalAcceleration(const math::RealVector3& ext_accel) {
  size_t num_vertices = mesh_->GetNumVertices();
  size_t n_dof_per_vertex = mesh_->GetNumDOFPerVertex();
  AX_THROW_IF_NE(n_dof_per_vertex, 3, "This overload only works for 3D mesh.");

  math::RealField3 ext_accel_field(3, static_cast<Index>(num_vertices));
  ext_accel_field.colwise() = ext_accel;
  SetExternalAcceleration(flatten(view_from_matrix(ext_accel_field)));
}

void TimeStepBase::UpdateEnergy() {
  problem_.UpdateEnergy();
}

void TimeStepBase::UpdateGradient() {
  problem_.UpdateGradient();
  prune_dirichlet_bc_.PruneGradient(problem_.GetGradient());
}

void TimeStepBase::UpdateHessian() {
  problem_.UpdateHessian();
  prune_dirichlet_bc_.Prune(*problem_.GetHessian());
}

void TimeStepBase::SetTimeStep(Real dt) {
  dt_ = dt;
  problem_.GetTerm("elasticity").scale_ = dt * dt;
  is_set_time_step_called_ = true;
}

void TimeStepBase::BeginStep() {
  // prepare the rhs.
  auto u_inertia = cache_inertia_->GetRhs();
  auto ext_accel = ext_accel_->ConstView();

  // u_inertia = u + dt v + dt2 a
  copy(u_inertia, u_back_->ConstView());                            // u
  math::buffer_blas::axpy(dt_, velocity_->ConstView(), u_inertia);  // dt v
  math::buffer_blas::axpy(dt_ * dt_, ext_accel, u_inertia);         // dt2 a

  prune_dirichlet_bc_.PruneVariable(u_inertia);
  copy(u_->View(), u_inertia);  // We set the initial guess to the inertia term.
  MarkCurrentSolutionUpdated();
}

void TimeStepBase::MarkCurrentSolutionUpdated() {
  problem_.MarkDirty();
}

void TimeStepBase::UpdateCurrentSolution(ConstRealBufferView u_current) {
  copy(u_->View(), u_current);
  MarkCurrentSolutionUpdated();
}

void TimeStepBase::SolveStep() {
  // Implement the backward euler with 1 newton step.
  auto start = utils::now();

  // We expect to decrease the |grad| to 0.01 * initial |grad|.
  optim2::Optimizer_Newton newton;
  newton.SetProblem(PrepareVariationalProblem());
  auto result = newton.Optimize({});
  AX_INFO("Newton step: {}", result.n_iter_);
  auto end = utils::now();
  AX_INFO("Solve time: {}", std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
}

void TimeStepBase::EndStep() {
  // update the velocity.
  auto [u, v, u_back] = make_view(u_, velocity_, u_back_);
  copy(v, u);                                // v <- u
  math::buffer_blas::axpy(-1.0, u_back, v);  // v <- u - u_back
  math::buffer_blas::scal(1.0 / dt_, v);     // v <- (u - u_back) / dt

  // backup the state.
  copy(u_back, u);
  AX_INFO("Displacement norm: {:12.6e}", math::buffer_blas::norm(u));
  AX_INFO("    Velocity norm: {:12.6e}", math::buffer_blas::norm(v));
}

void TimeStepBase::UpdatePruneDirichletBc() {
  prune_dirichlet_bc_.UpdateDbcValue();
}

void TimeStepBase::Step() {
  BeginStep();
  SolveStep();
  EndStep();
}

PruneDirichletBc& TimeStepBase::GetPruneDirichletBc() {
  return prune_dirichlet_bc_;
}

std::unique_ptr<TimeStepVariationalProblem> TimeStepBase::PrepareVariationalProblem() {
  return std::make_unique<TimeStepVariationalProblem>(this);
}

TimeStepVariationalProblem::TimeStepVariationalProblem(not_null<TimeStepBase*> timestep)
    : optim2::ProblemBase(timestep->problem_.GetState()->GetVariables()->View(),
                          timestep->problem_.GetGradient(),
                          timestep->problem_.GetHessian()),
      timestep_(timestep) {}

void TimeStepVariationalProblem::UpdateEnergy() {
  timestep_->UpdateEnergy();
  energy_ = timestep_->problem_.GetEnergy();
}

void TimeStepVariationalProblem::UpdateGradient() {
  timestep_->UpdateGradient();
}

void TimeStepVariationalProblem::UpdateHessian() {
  timestep_->UpdateHessian();
}

void TimeStepVariationalProblem::MarkVariableChanged() {
  timestep_->MarkCurrentSolutionUpdated();
}

void TimeStepBase::Compute() {
  // make sure the necessary parameters are set properly.
  if (!is_set_density_called_) {
    AX_THROW_RUNTIME_ERROR("should call SetDensity at least once.");
  } else if (!is_set_lame_called_) {
    AX_THROW_RUNTIME_ERROR("should call SetLame at least once.");
  } else if (!is_set_time_step_called_) {
    AX_THROW_RUNTIME_ERROR("should call SetTimeStep at least once.");
  }

  // prepare the tol_grad.
  if (tol_rel_grad_ < 1e-5) {
    AX_WARN("Relative Error is too small? got {:12.6e}", tol_rel_grad_);
  }

  math::RealMatrixX rhs(problem_.GetState()->GetNumDOFPerVertex(),
                        problem_.GetState()->GetNumVertices());
  rhs.setZero();
  rhs.row(0).setOnes();
  cache_inertia_->SetRhs(view_from_matrix(rhs));
  cache_inertia_->UpdateGradient();
  auto body_force_norm = math::buffer_blas::norm(cache_inertia_->GetGradient());
  tol_abs_grad_ = tol_rel_grad_ * body_force_norm;

  AX_INFO("Body force norm: {:12.6e} | Tol abs grad: {:12.6e}", body_force_norm, tol_abs_grad_);
}

void TimeStepBase::SetRelativeTolerance(Real tol_rel_grad) {
  tol_rel_grad_ = tol_rel_grad;
}

void TimeStepBase::SetOptions(utils::Options const& option) {
  AX_SYNC_OPT_IF(option, Real, tol_rel_grad) {
    if (tol_rel_grad_ <= math::epsilon<>) {
      AX_THROW_INVALID_ARGUMENT("Tol rel grad too small: {:12.6e}", tol_rel_grad_);
    }
  }

  if (auto y_it = option.find("youngs"), p_it = option.find("poisson");
      y_it != option.end() || p_it != option.end()) {
    if (y_it == option.end() || p_it == option.end()) {
      AX_THROW_INVALID_ARGUMENT("Youngs and Poisson must be set together.");
    }

    Real youngs = y_it->value().as_double();
    Real poisson = p_it->value().as_double();

    // compute lame
    auto lame = elasticity::compute_lame(youngs, poisson);
    SetLame(lame);
  }

  if (auto d_it = option.find("density"); d_it != option.end()) {
    SetDensity(d_it->value().as_double());
  }

  if (auto dt_it = option.find("dt"); dt_it != option.end()) {
    SetTimeStep(dt_it->value().as_double());
  }

  if (auto kind = option.find("elasticity_kind"); kind != option.end()) {
    auto name = kind->value().as_string().data();
    auto kind_actual = utils::reflect_enum<ElasticityKind>(name);
    if (! kind_actual) {
      AX_THROW_INVALID_ARGUMENT("Invalid elasticity_kind: {}", name);
    }
    SetElasticity(*kind_actual);
  }
}


} // namespace ax::fem