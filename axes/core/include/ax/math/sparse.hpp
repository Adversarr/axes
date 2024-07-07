#pragma once

#include <Eigen/SparseCore>

#include "ax/math/common.hpp"  // IWYU pragma: export
#include "ax/math/linalg.hpp"

namespace ax::math {

/**
 * @brief Alias for a sparse matrix with real values, column-major storage, and index type idx.
 */
using spmatr = Eigen::SparseMatrix<real, Eigen::RowMajor, idx>;

/**
 * @brief Alias for a triplet of real value, representing a coefficient in a sparse matrix.
 */
using sp_coeff = Eigen::Triplet<real, idx>;

/**
 * @brief Alias for a list of sparse coefficients.
 */
using sp_coeff_list = List<sp_coeff>;

/**
 * @brief Creates a sparse matrix with the specified number of rows and columns, using the given
 * coefficient list.
 *
 * @param rows The number of rows in the sparse matrix.
 * @param cols The number of columns in the sparse matrix.
 * @param coeff_list The list of coefficients to populate the sparse matrix.
 * @return The created sparse matrix.
 */
spmatr make_sparse_matrix(idx rows, idx cols, sp_coeff_list const& coeff_list);
spmatr make_sparse_matrix(idx rows, idx cols, std::vector<idx> const& row,
                          std::vector<idx> const& col, std::vector<real> const& val);

template <idx dim> spmatr kronecker_identity(spmatr A) {
  idx const rows = A.rows() * dim;
  idx const cols = A.cols() * dim;
  sp_coeff_list coeff_list;
  coeff_list.reserve(static_cast<size_t>(A.nonZeros()) * dim);
  for (idx k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      for (idx i = 0; i < dim; ++i) {
        coeff_list.push_back({it.row() * dim + i, it.col() * dim + i, it.value()});
      }
    }
  }
  return make_sparse_matrix(rows, cols, coeff_list);
}

template <typename Fn>
AX_FORCE_INLINE void spmatr_for_each(spmatr & A, Fn&& fn) {
  for (idx k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}

template <typename Fn>
AX_FORCE_INLINE void spmatr_for_each(const spmatr & A, Fn&& fn) {
  for (idx k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}


AX_FORCE_INLINE real norm(spmatr const& A, math::l1_t) {
  real norm = 0;
  real row_sum = 0;
  idx last_row = -1;
  spmatr_for_each(A, [&](idx row, idx, real val) {
    if (row != last_row) {
      norm = std::max(norm, row_sum);
      row_sum = 0;
      last_row = row;
    }
    row_sum += std::abs(val);
  });
  norm = std::max(norm, row_sum);
  return norm;
}

AX_FORCE_INLINE real norm(spmatr const& A, math::l2_t) {
  return A.norm();
}

}  // namespace ax::math
