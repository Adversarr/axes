#pragma once
#include "ax/optim/spsdm.hpp"

namespace ax::optim {

class DiagonalModification : public SpsdModificationBase {
public:
  virtual math::matxxr Modify(math::matxxr const& A) final;
  virtual math::sp_matxxr Modify(math::sp_matxxr const& A) final;

  Status SetOptions(utils::Opt const& options) final;
  utils::Opt GetOptions() const final;

  real additional_offset_{1e-6};
};

}