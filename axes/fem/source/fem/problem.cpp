#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include "ax/fem/problem.hpp"

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/buffer_blas.hpp"

namespace ax::fem {

ConstRealBufferView TermBase::GetGradient() const {
  return gradient_->ConstView();
}

const math::RealBlockMatrix& TermBase::GetHessian() const {
  return hessian_;
}

ConstSizeBufferView TermBase::GetConstraints() const {
  return constraints_->ConstView();
}

void TermBase::MarkDirty() {
  is_energy_up_to_date_ = false;
  is_gradient_up_to_date_ = false;
  is_hessian_up_to_date_ = false;
}

Problem::Problem(std::shared_ptr<State> state) : state_(std::move(state)) {
  auto var = state_->GetVariables();
  auto [ndof, nvert, _] = *var->Shape();
  auto device = state_->GetVariables()->Device();
  gradient_ = create_buffer<Real>(device, state_->GetVariables()->Shape());
  bsr_hessian_ = math::RealBlockMatrix(nvert, nvert, ndof, device);
  gradient_->SetBytes(0);
}

void Problem::AddTerm(std::string const& name, std::unique_ptr<TermBase> term) {
  TermInfo info;
  info.term_ = std::move(term);
  info.name_ = name;
  terms_.push_back(std::move(info));
}

Problem::TermInfo& Problem::GetTerm(std::string const& name) {
  for (auto& term : terms_) {
    if (term.name_ == name) {
      return term;
    }
  }
  AX_THROW_RUNTIME_ERROR("Term not found.");
}

bool Problem::HasTerm(std::string const& name) const {
  for (const auto& term : terms_) {
    if (term.name_ == name) {
      return true;
    }
  }
  return false;
}

bool Problem::RemoveTerm(std::string const& name) {
  auto it = std::find_if(terms_.begin(), terms_.end(), [name](const TermInfo& term) {
    return term.name_ == name;
  });
  if (it != terms_.end()) {
    it->term_ = std::move(terms_.back().term_);
    it->name_ = std::move(terms_.back().name_);
    it->gather_op_ = std::move(terms_.back().gather_op_);
    it->scale_ = terms_.back().scale_;
    terms_.pop_back();
    return true;
  }
  return false;
}

void Problem::UpdateGradient() {
  for (auto& term : terms_) {
    AX_TRACE("Updating gradient for term: {}", term.name_);
    term.term_->UpdateGradient();
  }

  gradient_->SetBytes(0);
  auto g_view = gradient_->View();
  for (auto& term : terms_) {
    AX_TRACE("Gathering gradient for term: {}", term.name_);
    auto grad = term.term_->GetGradient();
    math::buffer_blas::axpy(term.scale_, grad, g_view);
  }

  AX_TRACE("Gradient updated");
}

void Problem::UpdateHessian() {
  for (auto& term : terms_) {
    AX_TRACE("Updating Hessian for term: {}", term.name_);
    term.term_->UpdateHessian();
  }

  bsr_hessian_.Values()->SetBytes(0);
  for (auto& term : terms_) {
    AX_TRACE("Gathering Hessian for term: {}", term.name_);
    const auto& hess = term.term_->GetHessian();
    // TODO: Implement the gather operation.
    // term.gather_op_.Apply(hess.BlockValuesView(), bsr_hessian_.BlockValuesView(), 1.0, 1.0);
    math::buffer_blas::axpy(term.scale_, hess.Values()->View(), bsr_hessian_.Values()->View());
  }

  AX_TRACE("Hessian updated");
}

void Problem::InitializeHessianFillIn() {
  AX_ERROR("Not implemented.");
}

Real TermBase::GetEnergy() const { return energy_; }
} // namespace ax::fem