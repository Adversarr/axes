#include "ax/math/high_order/gather.hpp"
#ifdef AX_HAS_CUDA
#  include "./gather_gpu.cuh"
#endif
#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/for_each.hpp"

namespace ax::math {

static void do_host(ConstRealBufferView src, RealBufferView dst, ConstRealBufferView weights,
                    ConstSizeBufferView row_entries, ConstSizeBufferView col_indices, Real alpha,
                    Real beta, size_t n_output, size_t gather_dim) {
  auto job_1d = [&row_entries, &dst, &weights, &src, &col_indices, beta, alpha](size_t row) {
    const size_t row_begin = row_entries(row);
    const size_t row_end = row_entries(row + 1);
    Real sum = 0;
    for (size_t i = row_begin; i < row_end; ++i) {
      const size_t col = col_indices(i);
      sum += weights(i) * src(col);
    }
    dst(row) = alpha * sum + beta * dst(row);
  };

  auto job_2d
      = [&row_entries, &col_indices, &weights, &src, &dst, beta, alpha](size_t sub, size_t row) {
          const size_t row_begin = row_entries(row);
          const size_t row_end = row_entries(row + 1);
          Real sum_sub = 0;
          for (size_t i = row_begin; i < row_end; ++i) {
            const size_t col = col_indices(i);
            sum_sub += weights(i) * src(sub, col);
          }
          dst(sub, row) = alpha * sum_sub + beta * dst(sub, row);
        };

  auto job_3d = [&row_entries, &col_indices, &weights, &src, &dst, beta, alpha](
                    size_t si, size_t sj, size_t row) {
    const size_t row_begin = row_entries(row);
    const size_t row_end = row_entries(row + 1);
    Real sum_sub = 0;
    for (size_t i = row_begin; i < row_end; ++i) {
      const size_t col = col_indices(i);
      sum_sub += weights(i) * src(si, sj, col);
    }
    dst(si, sj, row) = alpha * sum_sub + beta * dst(si, sj, row);
  };

  if (gather_dim == 0) {
    if (n_output > 2048) {
      par_for_each_indexed(Dim{n_output}, job_1d);
    } else {
      for_each_indexed(Dim{n_output}, job_1d);
    }
  } else if (gather_dim == 1) {
    if (n_output > 2048) {
      par_for_each_indexed(Dim{src.Shape().X(), n_output}, job_2d);
    } else {
      for_each_indexed(Dim{src.Shape().X(), n_output}, job_2d);
    }
  } else {
    if (n_output > 2048) {
      par_for_each_indexed(Dim{src.Shape().X(), src.Shape().Y(), n_output}, job_3d);
    } else {
      for_each_indexed(Dim{src.Shape().X(), src.Shape().Y(), n_output}, job_3d);
    }
  }
}

void GatherAddOp::SetData(ConstRealBufferView weights, ConstSizeBufferView row_entries,
                          ConstSizeBufferView col_indices) {
  AX_THROW_IF_NE(weights.Shape().X(), n_gather_, "Weights shape is not correct.");
  AX_THROW_IF_NE(row_entries.Shape().X(), n_output_ + 1, "Row entries shape is not correct.");
  AX_THROW_IF_NE(col_indices.Shape().X(), n_gather_, "Column indices shape is not correct.");

  weights_ = ensure_buffer<Real>(weights_, device_, weights.Shape());
  row_entries_ = ensure_buffer<size_t>(row_entries_, device_, row_entries.Shape());
  col_indices_ = ensure_buffer<size_t>(col_indices_, device_, col_indices.Shape());
  copy(weights_->View(), weights);
  copy(row_entries_->View(), row_entries);
  copy(col_indices_->View(), col_indices);
}

void GatherAddOp::Apply(ConstRealBufferView src, RealBufferView dst, Real alpha, Real beta) const {
  const size_t gather_dim = is_1d(src.Shape()) ? 0 : is_2d(src.Shape()) ? 1 : 2;
  AX_THROW_IF_NE(src.Shape().sizes_[gather_dim], n_input_, "Input shape is not correct. expect {} got {}",
                  n_input_, src.Shape().sizes_[gather_dim]);
  AX_THROW_IF_NE(dst.Shape().sizes_[gather_dim], n_output_, "Output shape is not correct. expect {} got {}",
                  n_output_, dst.Shape().sizes_[gather_dim]);
  AX_THROW_IF(src.Device() != device_ || dst.Device() != device_, "Device mismatch.");

  if (device_ == BufferDevice::Device) {
#ifdef AX_HAS_CUDA
    // do device work.
    gather_device(src, dst, weights_->ConstView(), row_entries_->ConstView(),
                  col_indices_->ConstView(), alpha, beta, n_output_, gather_dim);
#else
    AX_THROW_RUNTIME_ERROR("CUDA is not enabled.");
#endif
  } else {
    // do host work.
    do_host(src, dst, weights_->View(), row_entries_->View(), col_indices_->View(), alpha, beta,
            n_output_, gather_dim);
  }
}

}  // namespace ax::math