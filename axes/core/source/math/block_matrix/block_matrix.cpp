#include "ax/math/block_matrix/block_matrix.hpp"

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/buffer/host_buffer.hpp"
#include "ax/core/excepts.hpp"
#include "ax/utils/ndrange.hpp"
#include "block_matrix_matmul_impl.hpp"

namespace ax::math {

void RealBlockMatrix::SetData(BufferView<const size_t> block_row_ptrs,
                              BufferView<const size_t> block_col_indices,
                              BufferView<const Real> block_values) {
  // check input is valid.
  AX_THROW_IF_FALSE(is_1d(block_row_ptrs.Shape()) && is_1d(block_col_indices.Shape())
                        && is_3d(block_values.Shape()),
                    "Invalid input shape. Require 1D, 1D, 3D buffer.");
  AX_THROW_IF(block_row_ptrs.Shape().X() != rows_ + 1, "Invalid block_row_ptrs size.");
  const size_t nnz = block_col_indices.Shape().X();
  AX_THROW_IF_FALSE(nnz == block_values.Shape().Z(),
                    "Invalid block_values size. col_indices={}, values={}", nnz,
                    block_values.Shape().Z());
  const size_t block_size = block_size_;
  AX_THROW_IF_FALSE(
      block_size == block_values.Shape().Y() && block_size == block_values.Shape().X(),
      "Invalid block_values buffer, expect square block. size={}x{}, expect={}",
      block_values.Shape().X(), block_values.Shape().Y(), block_size);

  // Ok valid, now create...
  BufferPtr<size_t> new_block_row_ptrs = block_row_ptrs_
                                             ? block_row_ptrs_->Clone(block_row_ptrs.Shape())
                                             : HostBuffer<size_t>::Create(block_row_ptrs.Shape());
  BufferPtr<size_t> new_block_col_indices
      = block_col_indices_ ? block_col_indices_->Clone(block_col_indices.Shape())
                           : HostBuffer<size_t>::Create(block_col_indices.Shape());
  BufferPtr<Real> new_block_values = block_values_ ? block_values_->Clone(block_values.Shape())
                                                   : HostBuffer<Real>::Create(block_values.Shape());

  // Copy data.
  copy(new_block_values->View(), block_values);
  copy(new_block_row_ptrs->View(), block_row_ptrs);
  copy(new_block_col_indices->View(), block_col_indices);

  block_values_ = std::move(new_block_values);
  block_row_ptrs_ = std::move(new_block_row_ptrs);
  block_col_indices_ = std::move(new_block_col_indices);
}

void RealBlockMatrix::RightMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst) const {
  const auto device = GetDevice();
  if (rhs.Device() != device || dst.Device() != device) {
    throw make_runtime_error("Device mismatch. rhs={}, dst={}, block_matrix={}", rhs.Device(),
                             dst.Device(), device);
  }
  // expect rhs is block_size X cols
  // expect dst is block_size X rows

  if (rhs.Shape().X() != block_values_->Shape().X()) {
    throw make_runtime_error("Invalid rhs shape. expect {}x{}, got {}x{}",
                             block_values_->Shape().X(), cols_, rhs.Shape().X(), rhs.Shape().Y());
  }

  if (dst.Shape().X() != block_values_->Shape().X()) {
    throw make_runtime_error("Invalid dst shape. expect {}x{}, got {}x{}",
                             block_values_->Shape().X(), rows_, dst.Shape().X(), dst.Shape().Y());
  }

  if (device == BufferDevice::Device) {
#ifdef AX_HAS_CUDA
    if (!rhs.IsContinuous() || !dst.IsContinuous()) {
      throw make_runtime_error("Buffer should be continuous for GPU matmul.");
    }
    details::block_matrix_matmul_gpu(rows_, cols_, block_values_->View(), block_row_ptrs_->View(),
                                     block_col_indices_->View(), rhs, dst);
#else
    throw make_runtime_error("CUDA is not enabled.");
#endif
  } else {
    details::block_matrix_matmul_cpu(rows_, cols_, block_values_->View(), block_row_ptrs_->View(),
                                     block_col_indices_->View(), rhs, dst);
  }
}

math::RealSparseMatrix RealBlockMatrix::ToSparseMatrix() const {
  // Convert to CSR format.
  math::SparseCOO coo;
  coo.reserve(NumNonZeroBlocks() * BlockSize() * BlockSize());

  auto row_ptrs = block_row_ptrs_->ConstView();
  auto value = block_values_->ConstView();
  auto col_indices = block_col_indices_->ConstView();
  auto block_size = BlockSize();

  for (size_t row = 0; row < rows_; ++row) {
    size_t block_curr = row_ptrs(row);
    size_t block_next = row_ptrs(row + 1);
    for (size_t block_id = block_curr; block_id < block_next; ++block_id) {
      size_t col = col_indices(block_id);
      for (size_t i = 0; i < block_size; ++i) {
        for (size_t j = 0; j < block_size; ++j) {
          Index r = static_cast<Index>(row * block_size + i);
          Index c = static_cast<Index>(col * block_size + j);
          coo.push_back({r, c, value(i, j, block_id)});
        }
      }
    }
  }

  return make_sparse_matrix(static_cast<Index>(Rows()), static_cast<Index>(Cols()), coo);
}

BufferView<const Real> RealBlockMatrix::BlockValuesView() const {
  return block_values_->View();
}

BufferView<const size_t> RealBlockMatrix::BlockColIndicesView() const {
  return block_col_indices_->View();
}

BufferView<const size_t> RealBlockMatrix::BlockRowPtrsView() const {
  return block_row_ptrs_->View();
}
}  // namespace ax::math