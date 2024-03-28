#pragma once

#include <Eigen/SparseCore>

#include "axes/core/common.hpp"

namespace ax::math {

/**
 * @brief Alias for a sparse matrix with real values, column-major storage, and index type idx.
 */
using sp_matxxr = Eigen::SparseMatrix<real, Eigen::RowMajor, idx>;

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
AX_FORCE_INLINE sp_matxxr make_sparse_matrix(idx rows, idx cols,
                                             sp_coeff_list const& coeff_list) {
  sp_matxxr mat(rows, cols);
  mat.setFromTriplets(coeff_list.begin(), coeff_list.end());
  return mat;
}

}  // namespace ax::math
