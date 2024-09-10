#include "ax/math/block_matrix/block_matrix.hpp"

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/copy.hpp"
#ifdef AX_HAS_CUDA
#  include "ax/core/buffer/device_buffer_raw.cuh"
#endif
#include "ax/core/buffer/host_buffer.hpp"
#include "ax/core/excepts.hpp"
#include "ax/math/block_matrix/details/matmul_impl.hpp"

namespace ax::math {

void RealBlockMatrix::SetData(BufferView<const int> block_row_ptrs,
                              BufferView<const int> block_col_indices,
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
  BufferPtr<int> new_block_row_ptrs, new_block_col_indices;
  BufferPtr<Real> new_block_values;

  // A little bit ugly. but works.
  if (block_row_ptrs_) {
    block_row_ptrs_->Resize(block_row_ptrs.Shape());
    new_block_row_ptrs = block_row_ptrs_;
  } else {
    if (device_ == BufferDevice::Host) {
      new_block_row_ptrs = HostBuffer<int>::Create(block_row_ptrs.Shape());
    } else {
#ifdef AX_HAS_CUDA
      new_block_row_ptrs = DeviceBufferRaw<int>::Create(block_row_ptrs.Shape());
#else
      throw make_runtime_error("CUDA is not enabled.");
#endif
    }
  }

  if (block_col_indices_) {
    block_col_indices_->Resize(block_col_indices.Shape());
    new_block_col_indices = block_col_indices_;
  } else {
    if (device_ == BufferDevice::Host) {
      new_block_col_indices = HostBuffer<int>::Create(block_col_indices.Shape());
    } else {
#ifdef AX_HAS_CUDA
      new_block_col_indices = DeviceBufferRaw<int>::Create(block_col_indices.Shape());
#else
      throw make_runtime_error("CUDA is not enabled.");
#endif
    }
  }

  if (block_values_) {
    block_values_->Resize(block_values.Shape());
    new_block_values = block_values_;
  } else {
    if (device_ == BufferDevice::Host) {
      new_block_values = HostBuffer<Real>::Create(block_values.Shape());
    } else {
#ifdef AX_HAS_CUDA
      new_block_values = DeviceBufferRaw<Real>::Create(block_values.Shape());
#else
      throw make_runtime_error("CUDA is not enabled.");
#endif
    }
  }

  // Copy data.
  copy(new_block_values->View(), block_values);
  copy(new_block_row_ptrs->View(), block_row_ptrs);
  copy(new_block_col_indices->View(), block_col_indices);

  block_values_ = new_block_values;
  block_row_ptrs_ = new_block_row_ptrs;
  block_col_indices_ = new_block_col_indices;
}

void RealBlockMatrix::RightMultiplyTo(BufferView<const Real> rhs, BufferView<Real> dst, Real alpha,
                                      Real beta) const {
  const auto device = GetDevice();
  EnsureMatDesc();
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
                                     block_col_indices_->View(), rhs, dst, alpha, beta,
                                     mat_desc_.get());
#else
    throw make_runtime_error("CUDA is not enabled.");
#endif
  } else {
    details::block_matrix_matmul_cpu(rows_, cols_, block_values_->View(), block_row_ptrs_->View(),
                                     block_col_indices_->View(), rhs, dst, alpha, beta,
                                     mat_desc_.get());
  }
}

math::RealSparseMatrix RealBlockMatrix::ToSparseMatrix() const {
  // Convert to CSR format.
  if (GetDevice() == BufferDevice::Device) {
    throw std::runtime_error(
        "Unable to transfer Device matrix to host directly, consider transfer the data first.");
  }

  math::SparseCOO coo;
  coo.reserve(NumNonZeroBlocks() * BlockSize() * BlockSize());

  auto row_ptrs = block_row_ptrs_->ConstView();
  auto value = block_values_->ConstView();
  auto col_indices = block_col_indices_->ConstView();
  auto block_size = BlockSize();

  for (size_t row = 0; row < rows_; ++row) {
    size_t block_curr = static_cast<size_t>(row_ptrs(row));
    size_t block_next = static_cast<size_t>(row_ptrs(row + 1));
    for (size_t block_id = block_curr; block_id < block_next; ++block_id) {
      size_t col = static_cast<size_t>(col_indices(block_id));
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

BufferView<const int> RealBlockMatrix::BlockColIndicesView() const {
  return block_col_indices_->View();
}

BufferView<const int> RealBlockMatrix::BlockRowPtrsView() const {
  return block_row_ptrs_->View();
}

void RealBlockMatrix::SetData(BufferPtr<int> block_row_ptrs, BufferPtr<int> block_col_indices,
                              BufferPtr<Real> block_values) {
  // check input is valid.
  AX_THROW_IF_FALSE(
      block_row_ptrs->Device() == device_ && block_col_indices->Device() == device_
          && block_values->Device() == device_,
      "Device mismatch. block_row_ptrs={}, block_col_indices={}, block_values={} expect={}",
      block_row_ptrs->Device(), block_col_indices->Device(), block_values->Device(), device_);

  AX_THROW_IF_FALSE(is_1d(block_row_ptrs->Shape()) && is_1d(block_col_indices->Shape())
                        && is_3d(block_values->Shape()),
                    "Invalid input shape. Require 1D, 1D, 3D buffer.");
  AX_THROW_IF(block_row_ptrs->Shape().X() != rows_ + 1, "Invalid block_row_ptrs size.");
  const size_t nnz = block_col_indices->Shape().X();
  AX_THROW_IF_FALSE(nnz == block_values->Shape().Z(),
                    "Invalid block_values size. col_indices={}, values={}", nnz,
                    block_values->Shape().Z());
  const size_t block_size = block_size_;
  AX_THROW_IF_FALSE(
      block_size == block_values->Shape().Y() && block_size == block_values->Shape().X(),
      "Invalid block_values buffer, expect square block. size={}x{}, expect={}",
      block_values->Shape().X(), block_values->Shape().Y(), block_size);

  block_row_ptrs_ = std::move(block_row_ptrs);
  block_col_indices_ = std::move(block_col_indices);
  block_values_ = std::move(block_values);
}

void RealBlockMatrix::Set(const RealBlockMatrix& other) {
  rows_ = other.rows_;
  cols_ = other.cols_;
  block_size_ = other.block_size_;
  // NOTE: If setdata throws, we will enter a bad state.

  SetData(other.block_row_ptrs_->View(), other.block_col_indices_->View(),
          other.block_values_->View());
}

void RealBlockMatrix::EnsureMatDesc() const {
#ifdef AX_HAS_CUDA
  if (GetDevice() == BufferDevice::Device) {
    if (!mat_desc_) {
      mat_desc_ = details::create_mat_desc_default();
    }
  }
#endif
}

}  // namespace ax::math