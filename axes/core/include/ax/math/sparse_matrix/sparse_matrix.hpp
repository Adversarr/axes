#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/common.hpp"

namespace ax::math {

class SparseMatrixBase {
public:
  virtual ~SparseMatrixBase() = default;

  virtual void /*NOLINT*/ RightMultiplyTo(ConstRealBufferView x, RealBufferView y, Real alpha = 1,
                                          Real beta = 0) const
      = 0;
};

}  // namespace ax::math