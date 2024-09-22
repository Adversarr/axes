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

Problem::Problem(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh)
    : state_(std::move(state)), mesh_(std::move(mesh)) {
  auto var = state_->GetVariables();
  auto [ndof, nvert, _] = *var->Shape();
  auto device = state_->GetVariables()->Device();
  gradient_ = create_buffer<Real>(device, state_->GetVariables()->Shape());
  hessian_ = std::make_shared<math::RealBlockMatrix>(nvert, nvert, ndof, device);
  gradient_->SetBytes(0);
}

Problem::TermInfo& Problem::AddTerm(std::string const& name, std::unique_ptr<TermBase> term) {
  Expects(term);
  TermInfo info;
  info.term_ = std::move(term);
  info.name_ = name;
  return terms_.emplace_back(std::move(info));
}

Problem::TermInfo& Problem::GetTerm(std::string const& name) {
  for (auto& term : terms_) {
    if (term.name_ == name) {
      return term;
    }
  }

  throw std::invalid_argument("Term not found.");
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

void Problem::UpdateEnergy() {
  for (auto& term : terms_) {
    term.term_->UpdateEnergy();
  }

  energy_ = 0;
  for (auto& term : terms_) {
    auto energy = term.term_->GetEnergy();
    energy_ += energy * term.scale_;
    // AX_INFO("{} => energy: {}", term.name_, energy);
  }
  // AX_INFO("Total energy: {}", energy_);
}

void Problem::UpdateGradient() {
  for (auto& term : terms_) {
    term.term_->UpdateGradient();
  }

  gradient_->SetBytes(0);
  auto g_view = gradient_->View();
  for (auto& term : terms_) {
    auto grad = term.term_->GetGradient();
    math::buffer_blas::axpy(term.scale_, grad, g_view);
    // AX_INFO("{} => norm of gradient: {}", term.name_, math::buffer_blas::norm(grad));
  }
}

void Problem::UpdateHessian() {
  for (auto& term : terms_) {
    term.term_->UpdateHessian();
  }

  hessian_->Values()->SetBytes(0);
  auto values_view = hessian_->Values()->View();
  for (auto& term : terms_) {
    const auto& hess = term.term_->GetHessian();
    // TODO: Implement the gather operation.
    // term.gather_op_.Apply(hess.BlockValuesView(), bsr_hessian_.BlockValuesView(), 1.0, 1.0);
    math::buffer_blas::axpy(term.scale_, hess.Values()->View(), values_view);

    // AX_INFO("norm of hessian: {}", math::buffer_blas::norm(hess.Values()->ConstView()));
  }
}

void Problem::InitializeHessianFillIn() {
  // TODO: Implement the correct operation, here we assumed that the gather operation is the same
  // and we just need to initialize the fill-in to the front one.

  if (terms_.empty()) {
    AX_THROW_RUNTIME_ERROR("No terms in the problem. not initializing Hessian fill-in at all.");
  }

  auto& term = terms_.front().term_;
  term->UpdateHessian();
  auto const& hess = term->GetHessian();
  hessian_->SetData(hess.RowPtrs()->ConstView(), hess.ColIndices()->ConstView(),
                    hess.Values()->ConstView());
  hessian_->MarkAsSymmetric();
  hessian_->Finish();
  hessian_->Values()->SetBytes(0);

  AX_INFO("RowPtrs: {} NNZ: {} total fillin: {}", hessian_->RowPtrs()->Shape().X() - 1,
          hessian_->ColIndices()->Shape().X(), prod(hessian_->Values()->Shape()));
}

Real TermBase::GetEnergy() const {
  return energy_;
}

void Problem::MarkDirty() {
  for (auto& term : terms_) {
    term.term_->MarkDirty();
  }
}

}  // namespace ax::fem