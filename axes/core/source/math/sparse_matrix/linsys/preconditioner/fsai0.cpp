#include "ax/math/sparse_matrix/linsys/preconditioner/fsai0.hpp"

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/gsl.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/utils/formatting.hpp"

namespace ax::math {

void compute_fsai_cpu(const RealCSRMatrix& mat, RealCSRMatrix& fact_inv) {
  // stores the fact_inv[rptr[i]:rptr[i+1]].
  auto row_ptr = fact_inv.RowPtrs()->ConstView();
  auto col_ind = fact_inv.ColIndices()->ConstView();
  auto value = fact_inv.Values()->View();
  auto mat_row = mat.RowPtrs()->ConstView();
  auto mat_col = mat.ColIndices()->ConstView();
  auto mat_val = mat.Values()->ConstView();
  auto find_original_value = [&](int row, int col) -> Real {
    auto row_start = static_cast<size_t>(mat_row(static_cast<size_t>(row)));
    auto row_end = static_cast<size_t>(mat_row(static_cast<size_t>(row) + 1));
    for (auto i = row_start; i < row_end; ++i) {
      if (mat_col(i) == col) {
        return mat_val(i);
      }
    }
    return 0;
  };

  auto job_of_row = [&](size_t row) mutable {
    auto row_start = static_cast<size_t>(row_ptr(row));
    // auto row_end = static_cast<size_t>(row_ptr(row + 1));
    Index row_size = row_ptr(row + 1) - row_ptr(row);
    AX_EXPECTS(row_size > 0 && "Invalid row size");
    RealMatrixX mat(row_size, row_size);
    RealVectorX b = RealVectorX::Unit(row_size, row_size - 1);

    for (int j = 0; j < row_size; ++j) {
      int g_j_coresp_col = col_ind(row_start + static_cast<size_t>(j));
      for (int i = 0; i < row_size; ++i) {
        // for (i, j) find the corresponding coefficient in original matrix.
        int g_i_coresp_col = col_ind(row_start + static_cast<size_t>(i));
        Real a_ij = find_original_value(g_j_coresp_col, g_i_coresp_col);
        mat(i, j) = a_ij;
      }
    }

    // solve the linear system.
    Eigen::Map<RealVectorX> x(value.Offset(static_cast<size_t>(row_start)), row_size);
    x.noalias() = mat.ldlt().solve(b);

    Real x_last = x(row_size - 1);
    x *= rsqrt(x_last);
  };

  size_t rows = fact_inv.Rows();
  if (rows > 10240) {
    par_for_each_indexed(Dim{rows}, job_of_row);
  } else {
    for_each_indexed(Dim{rows}, job_of_row);
  }
}

void prepare_fsai_cpu(const RealCSRMatrix& mat, RealCSRMatrix& fact_inv) {
  auto row_ptr = mat.RowPtrs()->ConstView();
  auto col_ind = mat.ColIndices()->ConstView();
  auto mat_val = mat.Values()->ConstView();

  // Extract the lower triangulare of the matrix.
  RealSparseCOO coo;
  for (size_t i = 0; i < mat.BlockedRows(); ++i) {
    int row_start = row_ptr(i);
    int row_end = row_ptr(i + 1);
    for (int j = row_start; j < row_end; ++j) {
      int col = col_ind(static_cast<size_t>(j));
      if (col <= static_cast<int>(i)) {
        coo.emplace_back(static_cast<int>(i), col, mat_val(static_cast<size_t>(j)));
      }
    }
  }

  fact_inv.SetFromTriplets(coo);
}

void GeneralSparsePreconditioner_FSAI0::AnalyzePattern() {
  AX_THROW_IF_NULL(mat_, "Matrix is not set.");
  auto device = mat_->Device();
  size_t rows = mat_->Rows(), cols = mat_->Cols();
  AX_THROW_IF_NE(rows, cols, "FSAI expect SPD matrix. rows={}, cols={}", rows, cols);
  fact_inv_ = std::make_unique<math::RealCSRMatrix>(rows, cols, BufferDevice::Host);
  temp_ = create_buffer<Real>(device, {rows});
}

void GeneralSparsePreconditioner_FSAI0::Factorize() {
  AX_THROW_IF_NULL(fact_inv_, "AnalyzePattern is not called.");
  auto device = mat_->Device();
  auto mat_csr = mat_->Transfer(BufferDevice::Host)->ToCSR();
  prepare_fsai_cpu(*mat_csr, *fact_inv_);
  compute_fsai_cpu(*mat_csr, *fact_inv_);
  math::RealSparseMatrix spm = fact_inv_->ToSparseMatrix();

  spm = spm * spm.transpose();
  spm.makeCompressed();
  // fact_inv_->SetData(view_from_raw_buffer(spm.outerIndexPtr(), {spm.outerSize()}),
  //                    view_from_raw_buffer(spm.innerIndexPtr(), {spm.nonZeros()}),
  //                    view_from_raw_buffer(spm.valuePtr(), {spm.nonZeros()}));

  // transfer the fact_inv_ to the original device.
  if (device == BufferDevice::Device) {
    fact_inv_ = std::unique_ptr<RealCSRMatrix>(
        static_cast<RealCSRMatrix*>(fact_inv_->Transfer(device).release()));
  }

  fact_inv_->Finish();
}

void GeneralSparsePreconditioner_FSAI0::Solve(ConstRealBufferView b, RealBufferView x) const {
  AX_THROW_IF_NULL(fact_inv_, "Factorize is not called.");
  AX_THROW_IF_FALSE(b.Shape() == x.Shape(), "Invalid shape. b={}, x={}", b.Shape(), x.Shape());
  AX_THROW_IF_FALSE(b.Device() == x.Device(), "Device mismatch. b={}, x={}", b.Device(),
                    x.Device());

  auto temp = temp_->View().Reshaped(b.Shape());

  // Solve on host, the approximation is G G.T
  const auto& g = *fact_inv_;
  g.TransposeMultiply(b, temp, 1, 0);  // temp = g^T * b
  g.Multiply(temp, x, 1, 0);           // x = g * temp = g * g^T * b
  // std::cout << "b norm: " << buffer_blas::norm(b) << std::endl;
  // std::cout << "temp norm: " << buffer_blas::norm(temp) << std::endl;
  // std::cout << "x norm: " << buffer_blas::norm(x) << std::endl;
}

}  // namespace ax::math