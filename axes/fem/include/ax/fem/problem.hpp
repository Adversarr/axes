#pragma once
#include <memory>  // std::unique_ptr

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/fem/mesh.hpp"
#include "ax/fem/state.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"
#include "ax/math/high_order/gather.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

class TermBase : public utils::Tunable {
public:
  // NOTE: It is derive's responsibility to initialize the buffers.
  explicit TermBase(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh)
      : state_(state), mesh_(mesh), is_gradient_up_to_date_(false), is_hessian_up_to_date_(false) {
        };

  virtual ~TermBase() = default;

  virtual void MarkDirty();
  virtual void UpdateEnergy() = 0;
  virtual void UpdateGradient() = 0;
  virtual void UpdateHessian() = 0;

  Real GetEnergy() const;
  ConstRealBufferView GetGradient() const;
  const math::RealBlockMatrix& GetHessian() const;
  ConstSizeBufferView GetConstraints() const;

protected:
  std::shared_ptr<State> state_;
  std::shared_ptr<Mesh> mesh_;
  Real energy_;                        ///< energy of the term
  BufferPtr<Real> gradient_;           ///< gradient of the term
  math::RealBlockMatrix hessian_;  ///< Hessian matrix in block sparse row format
  bool is_energy_up_to_date_;          ///< dirty bit.
  bool is_gradient_up_to_date_;        ///< dirty bit.
  bool is_hessian_up_to_date_;         ///< dirty bit.
  BufferPtr<size_t> constraints_;      ///< constraints of the term
};

class Problem {
public:
  struct TermInfo {
    std::unique_ptr<TermBase> term_;
    std::string name_;
    math::GatherAddOp gather_op_;  // WARN: currently, not used.
    Real scale_{1.0};
  };

  explicit Problem(std::shared_ptr<State> state);

  void AddTerm(std::string const& name, std::unique_ptr<TermBase> term);
  TermInfo& GetTerm(std::string const& name);
  bool HasTerm(std::string const& name) const;
  bool RemoveTerm(std::string const& name);

  // Update each term's Gradient, and gather into the global gradient.
  void UpdateGradient();

  // Update each term's Hessian, and gather into the global Hessian.
  void UpdateHessian();

  // Update TermInfo gather_op_.
  void InitializeHessianFillIn();

  ConstRealBufferView GetGradient() const { return gradient_->ConstView(); }

  const math::RealBlockMatrix& GetHessian() const { return bsr_hessian_; }

private:
  std::shared_ptr<State> state_;
  std::shared_ptr<Mesh> mesh_;
  std::vector<TermInfo> terms_;
  BufferPtr<Real> gradient_;
  math::RealBlockMatrix bsr_hessian_;
};

}  // namespace ax::fem