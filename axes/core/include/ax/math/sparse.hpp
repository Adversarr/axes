#pragma once

#include <Eigen/SparseCore>

#include "ax/math/common.hpp"  // IWYU pragma: export
#include "ax/math/linalg.hpp"

namespace ax::math {

/**
 * @brief Alias for a sparse matrix with real values, column-major storage, and index type Index.
 */
using spmatr = Eigen::SparseMatrix<real, Eigen::ColMajor, Index>;

/**
 * @brief Alias for a triplet of real value, representing a coefficient in a sparse matrix.
 */
using sp_coeff = Eigen::Triplet<real, Index>;

/**
 * @brief Alias for a list of sparse coefficients.
 */
using sp_coeff_list = std::vector<sp_coeff>;

/**
 * @brief Creates a sparse matrix with the specified number of rows and columns, using the given
 * coefficient list.
 *
 * @param rows The number of rows in the sparse matrix.
 * @param cols The number of columns in the sparse matrix.
 * @param coeff_list The list of coefficients to populate the sparse matrix.
 * @return The created sparse matrix.
 */
spmatr make_sparse_matrix(Index rows, Index cols, sp_coeff_list const& coeff_list);
spmatr make_sparse_matrix(Index rows, Index cols, std::vector<Index> const& row,
                          std::vector<Index> const& col, std::vector<real> const& val);

template <Index dim> spmatr kronecker_identity(spmatr A) {
  Index const rows = A.rows() * dim;
  Index const cols = A.cols() * dim;
  sp_coeff_list coeff_list;
  coeff_list.reserve(static_cast<size_t>(A.nonZeros()) * dim);
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      for (Index i = 0; i < dim; ++i) {
        coeff_list.push_back({it.row() * dim + i, it.col() * dim + i, it.value()});
      }
    }
  }
  return make_sparse_matrix(rows, cols, coeff_list);
}

template <typename Fn>
AX_FORCE_INLINE void spmatr_for_each(spmatr & A, Fn&& fn) {
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}

template <typename Fn> AX_FORCE_INLINE void spmatr_for_each(const spmatr& A, Fn fn) {
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}

template <typename Fn> AX_FORCE_INLINE void spmatr_for_each(spmatr& A, Fn fn) {
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}

/****************************** matrix norm ******************************/

AX_FORCE_INLINE real norm(spmatr const& A, math::l1_t) {
  real norm = 0;
  real row_sum = 0;
  Index last_row = -1;
  spmatr_for_each(A, [&](Index row, Index, real val) {
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

/****************************** inner product ******************************/
template <typename DerivedLeft, typename DerivedRight,
          typename = std::enable_if_t<DerivedLeft::IsVectorAtCompileTime && DerivedRight::IsVectorAtCompileTime>>
AX_FORCE_INLINE real inner(Eigen::MatrixBase<DerivedLeft> const& left, const spmatr& bilinear,
                           Eigen::MatrixBase<DerivedRight> const& right) {
  real total = static_cast<real>(0.0);
  spmatr_for_each(bilinear, [&](Index row, Index col, real value) { total += left[row] * right[col] * value; });
  return total;
}

}  // namespace ax::math
