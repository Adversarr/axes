#include "ax/math/sparse_matrix/linsys/preconditioner/fsai0.hpp"

#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/gsl.hpp"
#include "ax/math/utils/formatting.hpp"

namespace ax::math {

void GeneralSparsePreconditioner_FSAI0::AnalyzePattern() {
  // nothing to do.
  AX_THROW_IF_NULL(mat_, "Matrix is not set.");
}

void compute_fsai_cpu(const RealCSRMatrix& mat, RealCSRMatrix& fact_inv) {
  // stores the fact_inv[rptr[i]:rptr[i+1]].
  auto row_ptr = fact_inv.RowPtrs()->ConstView();
  auto col_ind = fact_inv.ColIndices()->ConstView();
  auto value = fact_inv.Values()->View();
  auto mat_row = mat.RowPtrs()->ConstView();
  auto mat_col = mat.ColIndices()->ConstView();
  auto mat_val = mat.Values()->ConstView();
  auto find_original_value = [&](int row, int col) -> Real {
    int row_start = mat_row(row);
    int row_end = mat_row(row + 1);
    for (int i = row_start; i < row_end; ++i) {
      if (mat_col(i) == col) {
        return mat_val(i);
      }
    }
    return 0;
  };

  auto job_of_row = [&](size_t row) mutable {
    int row_start = row_ptr(row);
    int row_end = row_ptr(row + 1);
    Index row_size = row_end - row_start;
    AX_EXPECTS(row_size > 0 && "Invalid row size");
    RealMatrixX mat(row_size, row_size);
    RealVectorX b = RealVectorX::Unit(row_size, row_size - 1);

    for (int j = 0; j < row_size; ++j) {
      int g_j_coresp_col = col_ind(static_cast<size_t>(row_start + j));
      for (int i = 0; i < row_size; ++i) {
        // for (i, j) find the corresponding coefficient in original matrix.
        int g_i_coresp_col = col_ind(static_cast<size_t>(row_start + i));
        Real a_ij = find_original_value(g_j_coresp_col, g_i_coresp_col);
        mat(i, j) = a_ij;
      }
    }

    // solve the linear system.
    Eigen::Map<RealVectorX> x(value.Offset(static_cast<size_t>(row_start)), row_size);
    x.noalias() = mat.ldlt().solve(b);
  };

  for (size_t i = 0; i < fact_inv.Rows(); ++i) {
    job_of_row(i);
  }

  // TODO: Solve D.
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
        coo.emplace_back(i, col, mat_val(static_cast<size_t>(j)));
      }
    }
  }

  fact_inv.SetFromTriplets(coo);
}

void GeneralSparsePreconditioner_FSAI0::Factorize() {
  auto device = mat_->Device();
  auto mat_csr = mat_->ToCSR(device);
  // TODO: implement the factorization.
  // topology is the lower triangular

  fact_inv_ = std::make_unique<math::RealCSRMatrix>(mat_csr->Rows(), mat_csr->Cols(), device);

  if (device == BufferDevice::Host) {
    prepare_fsai_cpu(*mat_csr, *fact_inv_);
    compute_fsai_cpu(*mat_csr, *fact_inv_);
  } else {
    AX_NOT_IMPLEMENTED();
  }
}

void GeneralSparsePreconditioner_FSAI0::Solve(ConstRealBufferView b, RealBufferView x) const {
  AX_THROW_IF_NULL(fact_inv_, "Factorize is not called.");
  AX_THROW_IF_FALSE(b.Shape().X() == x.Shape().X(), "Invalid shape. b={}, x={}", b.Shape(),
                    x.Shape());
  AX_THROW_IF_FALSE(b.Device() == x.Device(), "Device mismatch. b={}, x={}", b.Device(),
                    x.Device());

  if (b.Device() == BufferDevice::Host) {
    // Solve on host.
    AX_NOT_IMPLEMENTED();
  } else {
    // Solve on device.
    AX_NOT_IMPLEMENTED();
  }
}

}  // namespace ax::math