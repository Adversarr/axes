#include "./jacobi_impl.hpp"
#include "ax/core/buffer/for_each.hpp"

namespace ax::math::details {

void jacobi_precond_precompute_cpu(RealBufferView dst, const RealBlockMatrix& mat) {
  // Sequential:
  auto bv = mat.BlockValuesView();
  auto br = mat.BlockRowPtrsView();
  auto bc = mat.BlockColIndicesView();

  AX_THROW_IF_FALSE(dst.IsContinuous(), "dst must be continuous for memory mapping.");

  std::memset(dst.Data(), 0, sizeof(Real) * prod(dst.Shape()));

  size_t bs = mat.BlockSize();

  std::atomic<bool> has_found_zero_diag{false};
  auto job = [bs, bv, br, bc, &dst, &has_found_zero_diag](size_t row) {
    int row_start = br(row);
    int row_end = br(row + 1);

    for (size_t j = 0; j < bs; ++j) {
      dst(j, row) = 1.0;
    }

    bool found_diag = false;
    for (int block_idx = row_start; block_idx < row_end; ++block_idx) {
      int col = bc(static_cast<size_t>(block_idx));
      if (static_cast<int>(row) == col) {
        found_diag = true;
        for (size_t j = 0; j < bs; ++j) {
          dst(j, row) = 1.0 / bv(j, j, static_cast<size_t>(block_idx));
        }
      }
    }

    if_unlikely (!found_diag) {
      has_found_zero_diag.store(true, std::memory_order_relaxed);
    }
  };

  if (mat.BlockedRows() > 4096 * 16) {
    par_for_each_indexed(Dim{mat.BlockedRows()}, job);
  } else {
    for_each_indexed(Dim{mat.BlockedRows()}, job);
  }

  if (has_found_zero_diag) {
    AX_WARN("Zero diagonal block found in Jacobi preconditioner. (Will set to identity)");
  }
}

void jacobi_precond_solve_cpu(RealBufferView dst, ConstRealBufferView rhs,
                              ConstRealBufferView inv_diag) {
  size_t bs = inv_diag.Shape().X();
  size_t rows = inv_diag.Shape().Y();
  AX_THROW_IF_FALSE(dst.Shape().X() == bs, "dst.Shape().X() == bs");
  AX_THROW_IF_FALSE(dst.Shape().Y() == rows, "dst.Shape().Y() == rows");

  std::memset(dst.Data(), 0, sizeof(Real) * prod(dst.Shape()));

  auto job = [bs, &dst, &inv_diag, &rhs](size_t row) {
    for (size_t i = 0; i < bs; ++i) {
      dst(i, row) = rhs(i, row) * inv_diag(i, row);
    }
  };

  if (rows > 4096) {
    par_for_each_indexed(Dim{rows}, job);
  } else {
    for_each_indexed(Dim{rows}, job);
  }
}

}  // namespace ax::math::details