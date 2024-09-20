#pragma once

#ifdef AX_SPARSE_CSC_DEFAULT
#  define AX_SPARSE_MAJOR ColMajor
#else
#  ifndef AX_SPARSE_CSR_DEFAULT
#    define AX_SPARSE_CSR_DEFAULT
#  endif

#  define AX_SPARSE_MAJOR RowMajor
#endif

#include <Eigen/SparseCore>

#include "ax/math/common.hpp"  // IWYU pragma: export
#include "ax/math/linalg.hpp"

namespace ax::math {

constexpr Eigen::StorageOptions default_sparse_storage = Eigen::AX_SPARSE_MAJOR;

template <typename Scalar>
using SparseMatrix = Eigen::SparseMatrix<Scalar, default_sparse_storage, Index>;

/**
 * @brief Alias for a sparse matrix with real values, column-major storage, and index type Index.
 */
using RealSparseMatrix = Eigen::SparseMatrix<Real, default_sparse_storage, Index>;

/**
 * @brief Alias for a triplet of real value, representing a coefficient in a sparse matrix.
 */
using RealSparseEntry = Eigen::Triplet<Real, Index>;

/**
 * @brief Alias for a list of sparse coefficients.
 */
using RealSparseCOO = std::vector<RealSparseEntry>;

/**
 * @brief Creates a sparse matrix with the specified number of rows and columns, using the given
 * coefficient list.
 *
 * @param rows The number of rows in the sparse matrix.
 * @param cols The number of columns in the sparse matrix.
 * @param coeff_list The list of coefficients to populate the sparse matrix.
 * @return The created sparse matrix.
 */
RealSparseMatrix make_sparse_matrix(Index rows, Index cols, RealSparseCOO const& coeff_list);
RealSparseMatrix make_sparse_matrix(Index rows, Index cols, std::vector<Index> const& row,
                                    std::vector<Index> const& col, std::vector<Real> const& val);

template <int dim>
RealSparseMatrix kronecker_identity(RealSparseMatrix A) {
  Index const rows = A.rows() * dim;
  Index const cols = A.cols() * dim;
  RealSparseCOO coeff_list;
  coeff_list.reserve(static_cast<size_t>(A.nonZeros()) * dim);
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (RealSparseMatrix::InnerIterator it(A, k); it; ++it) {
      for (Index i = 0; i < dim; ++i) {
        coeff_list.push_back({it.row() * dim + i, it.col() * dim + i, it.value()});
      }
    }
  }
  return make_sparse_matrix(rows, cols, coeff_list);
}

template <typename Fn>
AX_FORCE_INLINE void for_each_entry(RealSparseMatrix& A, Fn&& fn) {
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (RealSparseMatrix::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}

template <typename Fn>
AX_FORCE_INLINE void for_each_entry(const RealSparseMatrix& A, Fn&& fn) {
  for (Index k = 0; k < A.outerSize(); ++k) {
    for (RealSparseMatrix::InnerIterator it(A, k); it; ++it) {
      fn(it.row(), it.col(), it.valueRef());
    }
  }
}

/****************************** matrix norm ******************************/

AX_FORCE_INLINE Real norm(RealSparseMatrix const& A, math::l1_t) {
  Real norm = 0;
  Real row_sum = 0;
  Index last_row = -1;
  for_each_entry(A, [&](Index row, Index, Real val) {
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

AX_FORCE_INLINE Real norm(RealSparseMatrix const& A, math::l2_t) {
  return A.norm();
}

/****************************** inner product ******************************/
template <typename DerivedLeft, typename DerivedRight,
          typename = std::enable_if_t<DerivedLeft::IsVectorAtCompileTime
                                      && DerivedRight::IsVectorAtCompileTime>>
AX_FORCE_INLINE Real inner(Eigen::MatrixBase<DerivedLeft> const& left,
                           const RealSparseMatrix& bilinear,
                           Eigen::MatrixBase<DerivedRight> const& right) {
  Real total = static_cast<Real>(0.0);
  for_each_entry(bilinear, [&](Index row, Index col, Real value) {
    total += left[row] * right[col] * value;
  });
  return total;
}

}  // namespace ax::math
