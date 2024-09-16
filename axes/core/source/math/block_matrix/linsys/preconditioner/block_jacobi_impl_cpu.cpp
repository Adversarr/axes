#include "ax/core/buffer/for_each.hpp"
#include "ax/core/excepts.hpp"
#include "block_jacobi_impl.hpp"
#include "ax/core/logging.hpp"
namespace ax::math::details {

void block_jacobi_precond_precompute_cpu(BufferView<Real> dst, const RealBlockMatrix& mat) {
  // Sequential:
  auto bv = mat.BlockValuesView();
  auto br = mat.BlockRowPtrsView();
  auto bc = mat.BlockColIndicesView();

  AX_THROW_IF_FALSE(dst.IsContinuous(), "dst must be continuous for memory mapping.");

  std::memset(dst.Data(), 0, sizeof(Real) * prod(dst.Shape()));

  size_t bs = mat.BlockSize();
  if (bs > AX_INVERSE_BLOCK_MAXSIZE) {
    throw std::runtime_error("Block size is too large for block Jacobi preconditioner.");
  }

  std::atomic<bool> has_found_zero_diag{false};
  // we assume that, there is no overlapping blocks in original matrix, therefore we do in parallel.
  auto job = [bs, bv, br, bc, &dst, &has_found_zero_diag](size_t row) {
    int row_start = br(row);
    int row_end = br(row + 1);

    for (size_t j = 0; j < bs; ++j) {
      for (size_t i = 0; i < bs; ++i) {
        dst(i, j, row) = (i == j) ? 1.0 : 0.0;
      }
    }

    bool found_diag = false;
    for (int block_idx = row_start; block_idx < row_end; ++block_idx) {
      int col = bc(static_cast<size_t>(block_idx));
      if (static_cast<int>(row) == col) {
        found_diag = true;
        for (size_t j = 0; j < bs; ++j) {
          for (size_t i = 0; i < bs; ++i) {
            dst(i, j, row) = bv(i, j, static_cast<size_t>(block_idx));
          }
        }
      }
    }

    if_unlikely (!found_diag) {
      has_found_zero_diag.store(true, std::memory_order_relaxed);
    }

    Real* ptr = dst.Offset(0, 0, row);
    if (bs == 2) {
      details::do_inplace_inverse<2>(ptr);
    } else if (bs == 3) {
      details::do_inplace_inverse<3>(ptr);
    } else if (bs == 4) {
      details::do_inplace_inverse<4>(ptr);
    }
  };
  if (mat.BlockedRows() > 4096) {
    par_for_each_indexed(Dim{mat.BlockedRows()}, job);
  } else {
    for_each_indexed(Dim{mat.BlockedRows()}, job);
  }

  if (has_found_zero_diag) {
    AX_WARN("Zero diagonal block found in block Jacobi preconditioner. (Will set to identity)");
  }
}

void block_jacobi_precond_eval_cpu(BufferView<Real> dst,          // a view of [bs, rows, 0]
                                   ConstRealBufferView rhs,       // a view of [bs, rows, 0]
                                   ConstRealBufferView inv_diag   // a view of [bs, bs, rows]
                                   ) {
  size_t bs = inv_diag.Shape().X();
  size_t rows = inv_diag.Shape().Z();
  AX_THROW_IF_FALSE(dst.Shape().X() == bs, "dst.Shape().X() == bs");
  AX_THROW_IF_FALSE(dst.Shape().Y() == rows, "dst.Shape().Y() == rows");
  AX_THROW_IF_FALSE(dst.Shape().Z() == 0, "dst.Shape().Z() == 0");

  std::memset(dst.Data(), 0, sizeof(Real) * prod(dst.Shape()));

  auto job = [bs = static_cast<Index>(bs), &dst, &inv_diag, &rhs](size_t row) {
    Eigen::Map<math::RealMatrixX> dst_block(dst.Offset(0, row, 0), bs, 1);
    Eigen::Map<const math::RealMatrixX> inv_diag_block(inv_diag.Offset(0, 0, row), bs, bs);
    Eigen::Map<const math::RealMatrixX> rhs_block(rhs.Offset(0, row, 0), bs, 1);
    dst_block = inv_diag_block * rhs_block;
  };

  if (rows > 4096) {
    par_for_each_indexed(Dim{rows}, job);
  } else {
    for_each_indexed(Dim{rows}, job);
  }
}

}  // namespace ax::math::details