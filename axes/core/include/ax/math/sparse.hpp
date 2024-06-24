#pragma once

#include <Eigen/SparseCore>

#include "ax/core/common.hpp"
#include "ax/math/common.hpp"

namespace ax::math {

/**
 * @brief Alias for a sparse matrix with real values, column-major storage, and index type idx.
 */
using spmatr = Eigen::SparseMatrix<real, Eigen::ColMajor, idx>;

/**
 * @brief Alias for a triplet of real value, representing a coefficient in a sparse matrix.
 */
using sp_coeff = Eigen::Triplet<real, idx>;

/**
 * @brief Alias for a list of sparse coefficients.
 */
using sp_coeff_list = List<sp_coeff>;

/**
 * @brief Creates a sparse matrix with the specified number of rows and columns, using the given coefficient list.
 * 
 * @param rows The number of rows in the sparse matrix.
 * @param cols The number of columns in the sparse matrix.
 * @param coeff_list The list of coefficients to populate the sparse matrix.
 * @return The created sparse matrix.
 */
AX_FORCE_INLINE spmatr make_sparse_matrix(idx rows, idx cols,
                                             sp_coeff_list const& coeff_list) {
  spmatr mat(rows, cols);
  mat.setFromTriplets(coeff_list.begin(), coeff_list.end());
  return mat;
}

template<idx dim>
spmatr kronecker_identity(spmatr A) {
  idx const rows = A.rows() * dim;
  idx const cols = A.cols() * dim;
  sp_coeff_list coeff_list;
  coeff_list.reserve(A.nonZeros() * dim);
  for (idx k = 0; k < A.outerSize(); ++k) {
    for (spmatr::InnerIterator it(A, k); it; ++it) {
      for (idx i = 0; i < dim; ++i) {
        coeff_list.push_back({it.row() * dim + i, it.col() * dim + i, it.value()});
      }
    }
  }
  return make_sparse_matrix(rows, cols, coeff_list);
}

}  // namespace ax::math
