#pragma once
#include <memory>  // std::unique_ptr

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/gsl.hpp"
#include "ax/fem/mesh.hpp"
#include "ax/fem/state.hpp"
#include "ax/math/high_order/gather.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

class TermBase : public utils::Tunable {
public:
  // NOTE: It is derive's responsibility to initialize the buffers.
  explicit TermBase(shared_not_null<State> state, shared_not_null<Mesh> mesh)
      : state_(state), mesh_(mesh) {};

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
  Real energy_;                         ///< energy of the term
  BufferPtr<Real> gradient_;            ///< gradient of the term
  math::RealBlockMatrix hessian_;       ///< Hessian matrix in block sparse row format
  bool is_energy_up_to_date_{false};    ///< dirty bit.
  bool is_gradient_up_to_date_{false};  ///< dirty bit.
  bool is_hessian_up_to_date_{false};   ///< dirty bit.
  BufferPtr<size_t> constraints_;       ///< constraints of the term
};

class Problem {
public:
  struct TermInfo {
    std::unique_ptr<TermBase> term_;
    std::string name_;
    math::GatherAddOp gather_op_;  // WARN: currently, not used.
    Real scale_{1.0};
  };

  explicit Problem(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh);

  TermInfo& AddTerm(std::string const& name, std::unique_ptr<TermBase> term);
  TermInfo& GetTerm(std::string const& name);
  bool HasTerm(std::string const& name) const;
  bool RemoveTerm(std::string const& name);

  // Update each term's Energy, and gather into the global energy.
  void UpdateEnergy();

  // Update each term's Gradient, and gather into the global gradient.
  void UpdateGradient();

  // Update each term's Hessian, and gather into the global Hessian.
  void UpdateHessian();

  // Update TermInfo gather_op_.
  void InitializeHessianFillIn();

  ConstRealBufferView GetGradient() const { return gradient_->ConstView(); }

  RealBufferView GetGradient() { return gradient_->View(); }

  std::shared_ptr<const math::RealBlockMatrix> GetHessian() const { return hessian_; }

  std::shared_ptr<math::RealBlockMatrix> GetHessian() { return hessian_; }

  Real GetEnergy() const { return energy_; }

  void MarkDirty();

  std::shared_ptr<State> GetState() { return state_; }

  std::shared_ptr<Mesh> GetMesh() { return mesh_; }

private:
  std::shared_ptr<State> state_;
  std::shared_ptr<Mesh> mesh_;
  std::vector<TermInfo> terms_;
  Real energy_;
  BufferPtr<Real> gradient_;
  std::shared_ptr<math::RealBlockMatrix> hessian_;
};

}  // namespace ax::fem