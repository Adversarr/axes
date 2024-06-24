#pragma once
#include "ax/optim/spsdm.hpp"

namespace ax::optim {

class DiagonalModification : public SpsdModificationBase {
public:
  virtual math::matxxr Modify(math::matxxr const& A) final;
  virtual math::spmatr Modify(math::spmatr const& A) final;

  void SetOptions(utils::Options const& options) final;
  utils::Options GetOptions() const final;

  real additional_offset_{1e-6};
};

}
