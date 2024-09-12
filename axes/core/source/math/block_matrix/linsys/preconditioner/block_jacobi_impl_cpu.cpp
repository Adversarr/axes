#include "ax/core/buffer/for_each.hpp"
#include "block_jacobi_impl.hpp"

namespace ax::math::details {

void block_jacobi_precond_precompute_cpu(BufferView<Real> dst, const RealBlockMatrix& mat,
                                         void* /* mat_desc_type_erased */
) {
  // Sequential:
  auto bv = mat.BlockValuesView();
  auto br = mat.BlockRowPtrsView();
  auto bc = mat.BlockColIndicesView();

  AX_THROW_IF_FALSE(dst.IsContinuous(), "dst must be continuous for memory mapping.");

  std::memset(dst.Data(), 0, sizeof(Real) * prod(dst.Shape()));

  size_t bs = mat.BlockSize();
  // we assume that, there is no overlapping blocks in original matrix, therefore we do in parallel.
  par_for_each_indexed(Dim{mat.BlockedRows()}, [bs, bv, br, bc, &dst](size_t row) {
    int row_start = br(row);
    int row_end = br(row + 1);

    for (size_t j = 0; j < bs; ++j) {
      for (size_t i = 0; i < bs; ++i) {
        dst(i, j, row) = (i == j) ? 1.0 : 0.0;
      }
    }

    for (int block_idx = row_start; block_idx < row_end; ++block_idx) {
      int col = bc(static_cast<size_t>(block_idx));
      if (static_cast<int>(row) == col) {
        for (size_t j = 0; j < bs; ++j) {
          for (size_t i = 0; i < bs; ++i) {
            dst(i, j, row) = bv(i, j, static_cast<size_t>(block_idx));
          }
        }
      }
    }

    Eigen::Map<math::RealMatrixX> inv_diag(dst.Offset(0, 0, row), static_cast<Index>(bs),
                                           static_cast<Index>(bs));
    // Perform in-place LDLT decomposition.
    Eigen::LDLT<Eigen::Ref<math::RealMatrixX>> ldlt_inplace(inv_diag);
  });
}

void block_jacobi_precond_eval_cpu(BufferView<Real> dst,          // a view of [bs, rows, 0]
                                   ConstRealBufferView rhs,       // a view of [bs, rows, 0]
                                   ConstRealBufferView inv_diag,  // a view of [bs, bs, rows]
                                   void* mat_desc_type_erased) {
  AX_UNUSED(mat_desc_type_erased);

  size_t bs = inv_diag.Shape().X();
  size_t rows = inv_diag.Shape().Z();
  AX_THROW_IF_FALSE(dst.Shape().X() == bs, "dst.Shape().X() == bs");
  AX_THROW_IF_FALSE(dst.Shape().Y() == rows, "dst.Shape().Y() == rows");
  AX_THROW_IF_FALSE(dst.Shape().Z() == 0, "dst.Shape().Z() == 0");

  std::memset(dst.Data(), 0, sizeof(Real) * prod(dst.Shape()));

  // Run in Parallel.
  par_for_each_indexed(Dim{rows}, [bs = static_cast<Index>(bs), &dst, &inv_diag, &rhs](size_t row) {
    Eigen::Map<math::RealMatrixX> dst_block(dst.Offset(0, row, 0), bs, 1);
    Eigen::Map<const math::RealMatrixX> inv_diag_block(inv_diag.Offset(0, 0, row), bs, bs);
    Eigen::Map<const math::RealMatrixX> rhs_block(rhs.Offset(0, row, 0), bs, 1);
    dst_block = inv_diag_block * rhs_block;
  });
}

}  // namespace ax::math::details