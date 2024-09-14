#pragma once
#include "ax/core/buffer/buffer_view.hpp"
namespace ax::math {
void gather_device(ConstRealBufferView src, RealBufferView dst,
                   ConstRealBufferView weights, ConstSizeBufferView row_entries,
                   ConstSizeBufferView col_indices, Real alpha, Real beta,
                   size_t n_output, size_t gather_dim);
}