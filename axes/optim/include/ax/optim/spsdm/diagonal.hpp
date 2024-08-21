#pragma once
#include "ax/optim/spsdm.hpp"

namespace ax::optim {

class DiagonalModification : public SpsdModificationBase {
public:
  virtual math::RealMatrixX Modify(math::RealMatrixX const& A) final;
  virtual math::RealSparseMatrix Modify(math::RealSparseMatrix const& A) final;

  void SetOptions(utils::Options const& options) final;
  utils::Options GetOptions() const final;

  Real additional_offset_{1e-6};
};

}
