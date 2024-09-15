#pragma once
#include "ax/core/buffer/buffer_view.hpp"

namespace ax::math {

class GatherAddOp {
public:
  GatherAddOp() = default;
  AX_DECLARE_CONSTRUCTOR(GatherAddOp, default, default);

  GatherAddOp(size_t n_in, size_t n_out, size_t n_gather, BufferDevice device = BufferDevice::Host)
      : device_(device), n_input_(n_in), n_output_(n_out), n_gather_(n_gather) {}

  void SetData(ConstRealBufferView weights, ConstSizeBufferView row_entries,
               ConstSizeBufferView col_indices);

  /**
   * @brief Gather src into dst. Perform:
   *            dst = beta * dst + alpha * weights * src
   *        as if it is just a SPMV operation.
   * @note  Will assume the last valid dimension of src is the dimension to gather.
   *
   * @param src [...,  n_input]
   * @param dst [..., n_output] ... is same as src
   */
  void Apply(ConstRealBufferView src, RealBufferView dst, Real alpha = 1, Real beta = 0) const;

private:
  BufferPtr<Real> weights_;        ///< [n_input], weight for gather
  BufferPtr<size_t> row_entries_;  ///< [n_output + 1], row entries for gather, like a CSR matrix
  BufferPtr<size_t> col_indices_;  ///< [n_input], column indices for gather
  BufferDevice device_;            ///< device for buffer
  size_t n_input_;                 ///< number of input
  size_t n_output_;                ///< number of output
  size_t n_gather_;                ///< number of gather
};

}  // namespace ax::math