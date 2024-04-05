#pragma once
#include "ax/optim/spsdm.hpp"
namespace ax::optim {

class EigenvalueModification : public SpsdModificationBase {
public:
  virtual StatusOr<math::matxxr> Modify(math::matxxr const& A) final;
  virtual StatusOr<math::sp_matxxr> Modify(math::sp_matxxr const& A) final;

  Status SetOptions(utils::Opt const& options) final;
  utils::Opt GetOptions() const final;

  real min_eigval_{1e-6};
};

}  // namespace ax::optim