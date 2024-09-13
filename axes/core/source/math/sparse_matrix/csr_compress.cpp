#include "ax/math/sparse_matrix/csr_compress.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_default.hpp"
#include "csr_compress_impl.hpp"

namespace ax::math {

RealSparseMatrixCompressed::RealSparseMatrixCompressed(const RealSparseMatrix& mat,
                                                       BufferDevice device) {
  rows_ = static_cast<size_t>(mat.rows());
  cols_ = static_cast<size_t>(mat.cols());
  device_ = device;

  RealSparseCOO coo;
  math::for_each_entry(mat, [&](size_t row, size_t col, Real value) {
    coo.emplace_back(row, col, value);
  });
  SetFromTriplets(coo);
}

void RealSparseMatrixCompressed::RightMultiplyTo(ConstRealBufferView x, RealBufferView y,
                                                 Real alpha, Real beta) const {
  auto device = device_;
  if (!is_same_device(x, y) || x.Device() != device) {
    throw make_runtime_error("Device mismatch. x={}, y={}, matrix={}", x.Device(), y.Device(),
                             device);
  }

  if (x.Shape() != y.Shape()) {
    throw make_runtime_error("The shape of x and y should be the same. x={}, y={}", x.Shape(),
                             y.Shape());
  }

  if (device == BufferDevice::Host) {
    details::compute_csr_spmv_cpu(x, y, alpha, beta, row_ptrs_->View(), col_indices_->View(),
                                  values_->View());
  } else {
    details::compute_csr_spmv_gpu(x, y, alpha, beta, mat_descr_);
  }
}

void RealSparseMatrixCompressed::SetData(BufferView<const int> row_ptrs,
                                         BufferView<const int> col_indices,
                                         BufferView<const Real> values) {
  if (!is_1d(row_ptrs.Shape()) || !is_1d(col_indices.Shape()) || !is_1d(values.Shape())) {
    throw std::runtime_error("All the input should be 1D array.");
  }

  size_t nnz = values.Shape().X();
  if (col_indices.Shape().X() != nnz) {
    throw make_runtime_error(
        "The size of col_indices should be the same as the number of values. col_ind={}, values={}",
        col_indices.Shape().X(), nnz);
  }

  if (values_) {
    values_->Resize(values.Shape());
  } else {
    values_ = create_buffer<Real>(device_, values.Shape());
  }

  if (row_ptrs_) {
    row_ptrs_->Resize(row_ptrs.Shape());
  } else {
    row_ptrs_ = create_buffer<int>(device_, row_ptrs.Shape());
  }

  if (col_indices_) {
    col_indices_->Resize(col_indices.Shape());
  } else {
    col_indices_ = create_buffer<int>(device_, col_indices.Shape());
  }
  mat_descr_.reset();

  // I assume that your input is valid.
  copy(row_ptrs_->View(), row_ptrs);
  copy(col_indices_->View(), col_indices);
  copy(values_->View(), values);

  if (device_ == BufferDevice::Device) {
    mat_descr_ = details::create_csr_compress_desc_gpu(row_ptrs_->View(), col_indices_->View(),
                                                       values_->View(), rows_, cols_);
  } else {
    mat_descr_.reset();
  }
}

void RealSparseMatrixCompressed::SetFromTriplets(const RealSparseCOO& coo) {
  // Input coo is always on host.
  // sort it.
  RealSparseCOO sorted_coo = coo;
  auto less = [](const RealSparseEntry& left, const RealSparseEntry& right) -> bool {
    // sort by row first, then by column.
    if (left.row() < right.row()) {
      return true;
    } else if (left.row() == right.row()) {
      return left.col() < right.col();
    } else {
      return false;
    }
  };
  std::sort(sorted_coo.begin(), sorted_coo.end(), less);

  // Do copy
  auto row_ptrs = create_buffer<int>(BufferDevice::Host, {rows_ + 1});
  auto col_indices = create_buffer<int>(BufferDevice::Host, {sorted_coo.size()});
  auto values = create_buffer<Real>(BufferDevice::Host, {sorted_coo.size()});
  auto [r, c, v] = make_view(row_ptrs, col_indices, values);
  row_ptrs->SetBytes(0);

  r(0) = 0;
  int last_row = 0;
  size_t cnt_row = 0;
  for (size_t i = 0; i < sorted_coo.size(); ++i) {
    const auto& entry = sorted_coo[i];

    if (entry.row() != last_row) {
      r(cnt_row + 1) = static_cast<int>(i);
      last_row = static_cast<int>(entry.row());
      ++cnt_row;
    }

    c(i) = static_cast<int>(entry.col());
    v(i) = entry.value();
  }
  r(cnt_row + 1) = static_cast<int>(sorted_coo.size());

  SetData(r, c, v);
}

}  // namespace ax::math