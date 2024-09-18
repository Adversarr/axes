#include "ax/math/sparse_matrix/block_matrix.hpp"

#include <set>

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#ifdef AX_HAS_CUDA
#  include "ax/core/buffer/device_buffer.cuh"
#endif
#include "ax/core/buffer/host_buffer.hpp"
#include "ax/core/excepts.hpp"
#include "ax/math/sparse_matrix/details/matmul_impl.hpp"

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
  if (row_ptrs_) {
    row_ptrs_->Resize(block_row_ptrs.Shape());
    new_block_row_ptrs = row_ptrs_;
  } else {
    if (device_ == BufferDevice::Host) {
      new_block_row_ptrs = HostBuffer<int>::Create(block_row_ptrs.Shape());
    } else {
#ifdef AX_HAS_CUDA
      new_block_row_ptrs = DeviceBuffer<int>::Create(block_row_ptrs.Shape());
#else
      AX_THROW_RUNTIME_ERROR("CUDA is not enabled.");
#endif
    }
  }

  if (col_indices_) {
    col_indices_->Resize(block_col_indices.Shape());
    new_block_col_indices = col_indices_;
  } else {
    if (device_ == BufferDevice::Host) {
      new_block_col_indices = HostBuffer<int>::Create(block_col_indices.Shape());
    } else {
#ifdef AX_HAS_CUDA
      new_block_col_indices = DeviceBuffer<int>::Create(block_col_indices.Shape());
#else
      AX_THROW_RUNTIME_ERROR("CUDA is not enabled.");
#endif
    }
  }

  if (values_) {
    values_->Resize(block_values.Shape());
    new_block_values = values_;
  } else {
    if (device_ == BufferDevice::Host) {
      new_block_values = HostBuffer<Real>::Create(block_values.Shape());
    } else {
#ifdef AX_HAS_CUDA
      new_block_values = DeviceBuffer<Real>::Create(block_values.Shape());
#else
      AX_THROW_RUNTIME_ERROR("CUDA is not enabled.");
#endif
    }
  }

  // Copy data.
  copy(new_block_values->View(), block_values);
  copy(new_block_row_ptrs->View(), block_row_ptrs);
  copy(new_block_col_indices->View(), block_col_indices);

  values_ = new_block_values;
  row_ptrs_ = new_block_row_ptrs;
  col_indices_ = new_block_col_indices;
}

void RealBlockMatrix::Multiply(BufferView<const Real> rhs, BufferView<Real> dst, Real alpha,
                               Real beta) const {
  const auto device = Device();
  if (rhs.Device() != device || dst.Device() != device) {
    AX_THROW_RUNTIME_ERROR("Device mismatch. rhs={}, dst={}, block_matrix={}", rhs.Device(),
                           dst.Device(), device);
  }
  // expect rhs is block_size X cols
  // expect dst is block_size X rows

  if (rhs.Shape().X() != values_->Shape().X()) {
    AX_THROW_RUNTIME_ERROR("Invalid rhs shape. expect {}x{}, got {}x{}", values_->Shape().X(),
                           cols_, rhs.Shape().X(), rhs.Shape().Y());
  }

  if (dst.Shape().X() != values_->Shape().X()) {
    AX_THROW_RUNTIME_ERROR("Invalid dst shape. expect {}x{}, got {}x{}", values_->Shape().X(),
                           rows_, dst.Shape().X(), dst.Shape().Y());
  }

  if (device == BufferDevice::Device) {
#ifdef AX_HAS_CUDA
    if (!rhs.IsContinuous() || !dst.IsContinuous()) {
      AX_THROW_RUNTIME_ERROR("Buffer should be continuous for GPU matmul.");
    }
    details::block_matrix_matmul_gpu(rows_, cols_, values_->View(), row_ptrs_->View(),
                                     col_indices_->View(), rhs, dst, alpha, beta, mat_desc_);
#else
    AX_THROW_RUNTIME_ERROR("CUDA is not enabled.");
#endif
  } else {
    details::block_matrix_matmul_cpu(rows_, cols_, values_->View(), row_ptrs_->View(),
                                     col_indices_->View(), rhs, dst, alpha, beta, mat_desc_);
  }
}

math::RealSparseMatrix RealBlockMatrix::ToSparseMatrix() const {
  // Convert to CSR format.
  if (Device() == BufferDevice::Device) {
    AX_THROW_RUNTIME_ERROR(
        "Unable to transfer Device matrix to host directly, consider transfer the data first.");
  }

  math::RealSparseCOO coo;
  coo.reserve(prod(values_->Shape()));

  auto row_ptrs = row_ptrs_->ConstView();
  auto value = values_->ConstView();
  auto col_indices = col_indices_->ConstView();
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

void RealBlockMatrix::TransposeMultiply(ConstRealBufferView x, RealBufferView y, Real alpha,
                                        Real beta) const {
  if (is_symm_) {
    Multiply(x, y, alpha, beta);
    return;
  }
  // TODO: Implement this.
  AX_THROW_RUNTIME_ERROR("Not implemented.");
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

  row_ptrs_ = std::move(block_row_ptrs);
  col_indices_ = std::move(block_col_indices);
  values_ = std::move(block_values);
}

void RealBlockMatrix::Set(const RealBlockMatrix& other) {
  rows_ = other.rows_;
  cols_ = other.cols_;
  block_size_ = other.block_size_;
  // NOTE: If setdata throws, we will enter a bad state.

  SetData(other.row_ptrs_->View(), other.col_indices_->View(), other.values_->View());
}

void RealBlockMatrix::SetFromBlockedTriplets(ConstIntBufferView row, ConstIntBufferView col,
                                             ConstRealBufferView values) {
  // check input is valid.
  AX_THROW_IF_FALSE(is_1d(row.Shape()) && is_1d(col.Shape()) && is_3d(values.Shape()),
                    "Invalid input shape. Require 1D, 1D, 3D buffer.");
  AX_THROW_IF(row.Shape().X() != col.Shape().X(), "Invalid row/col size.");
  const size_t b_cnt = values.Shape().Z();
  AX_THROW_IF_FALSE(b_cnt == row.Shape().X(), "Invalid values size. row={}, values={}",
                    row.Shape().X(), b_cnt);
  if (row.Device() != BufferDevice::Host || col.Device() != BufferDevice::Host
      || values.Device() != BufferDevice::Host) {
    AX_THROW_RUNTIME_ERROR("Only support host buffer for now.");
  }

  // Ok valid, now create...
  struct BlockEntry {
    int row_;
    int col_;
    size_t block_id_;
  };

  std::vector<BlockEntry> entries;
  entries.reserve(b_cnt);
  for (size_t i = 0; i < b_cnt; ++i) {
    entries.push_back({row(i), col(i), i});
  }

  // Sort the entries.
  std::sort(entries.begin(), entries.end(), [](const BlockEntry& a, const BlockEntry& b) {
    return a.row_ < b.row_ || (a.row_ == b.row_ && a.col_ < b.col_);
  });

  size_t nnz = [&entries]() {
    std::set<std::pair<int, int>> unique_entries;
    for (const auto& e : entries) {
      unique_entries.insert({e.row_, e.col_});
    }
    return unique_entries.size();
  }();

  // Now we can create the block matrix.
  auto rb = create_buffer<int>(BufferDevice::Host, {rows_ + 1});
  auto cb = create_buffer<int>(BufferDevice::Host, {nnz});
  auto vb = create_buffer<Real>(BufferDevice::Host, {block_size_, block_size_, nnz});
  auto [r, c, v] = make_view(rb, cb, vb);
  rb->SetBytes(0);
  vb->SetBytes(0);

  // Scatter the values.
  int this_row = 0, this_col = 0;
  size_t cnt_nnz = 0;
  for (size_t i = 0; i < b_cnt; ++i) {
    c(cnt_nnz) = entries[i].col_;
    this_row = entries[i].row_;
    this_col = entries[i].col_;
    r(static_cast<size_t>(this_row) + 1) += 1;
    while (entries[i].row_ == this_row && entries[i].col_ == this_col && i < b_cnt) {
      for (size_t j = 0; j < block_size_; ++j) {
        for (size_t k = 0; k < block_size_; ++k) {
          v(j, k, cnt_nnz) += values(j, k, entries[i].block_id_);
        }
      }
      ++i;
    }
    if (i < b_cnt) {
      --i;
    }
    ++cnt_nnz;
  }
  AX_DCHECK(cnt_nnz == nnz, "Invalid nnz count. expect={}, got={}", nnz, cnt_nnz);

  // Perform a prefix sum to get the ptr.
  for (size_t i = 1; i < rows_ + 1; ++i) {
    r(i) += r(i - 1);
  }

  SetData(r, c, v);
}

void RealBlockMatrix::Finish() {
#ifdef AX_HAS_CUDA
  if (Device() == BufferDevice::Device) {
    if (!mat_desc_) {
      mat_desc_ = details::create_bsr_mat_desc(row_ptrs_->View(), col_indices_->View(),
                                               values_->View(), rows_, cols_);
    }
  }
#endif
}

}  // namespace ax::math