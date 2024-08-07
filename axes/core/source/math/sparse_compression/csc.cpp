#include "ax/math/sparse_compression/csc.hpp"

#include "ax/core/echo.hpp"
#include "ax/core/excepts.hpp"

namespace ax::math {

// For csc matrix,
//  The outer index is the column index.
//  The inner index is the row index.
//  The values are the non-zero values.

void SparseCompressMatrix_CSC::Assign(spmatr const& A) {
  // Assign the inner index and values.
  AssignSparsePattern(A);
  AssignSparsePatternUnchanged(A);
  // Check the correctness of the assignment.
  size_t nc = static_cast<size_t>(cols_);
  AX_DCHECK(static_cast<size_t>(outer_index_[nc]) == inner_index_.size())
      << outer_index_[nc] << " " << inner_index_.size();
  AX_DCHECK(static_cast<size_t>(outer_index_[nc]) == values_.size()) << outer_index_[nc] << " " << values_.size();
}

void SparseCompressMatrix_CSC::AssignSparsePattern(spmatr const& A) {
  rows_ = A.rows();
  cols_ = A.cols();
  values_.resize(static_cast<size_t>(A.nonZeros()));

  // Count the number of non-zero elements in each column.
  col_nnz.resize(static_cast<size_t>(cols_), 0);
  for (idx i = 0; i < A.outerSize(); ++i) {
    for (spmatr::InnerIterator it(A, i); it; ++it) {
      col_nnz[static_cast<size_t>(it.col())]++;
    }
  }

  // Compute the outer index.
  outer_index_.resize(static_cast<size_t>(cols_ + 1));
  inner_index_.resize(static_cast<size_t>(A.nonZeros()));
  outer_index_[0] = 0;
  for (size_t i = 0; i < static_cast<size_t>(cols_); ++i) {
    outer_index_[i + 1] = outer_index_[i] + col_nnz[i];
  }

  // Assign the inner index
  col_nnz.resize(static_cast<size_t>(cols_), 0);
  for (idx i = 0; i < A.outerSize(); ++i) {
    for (spmatr::InnerIterator it(A, i); it; ++it) {
      size_t col = static_cast<size_t>(it.col());
      size_t idx = static_cast<size_t>(outer_index_[col] + col_nnz[col]);
      inner_index_[idx] = it.row();
      col_nnz[col]++;
    }
  }
}

void SparseCompressMatrix_CSC::AssignSparsePatternUnchanged(spmatr const& A) {
  AX_THROW_IF_TRUE(rows_ != A.rows() || cols_ != A.cols(), "The matrix size is different.");
  std::fill(col_nnz.begin(), col_nnz.end(), 0);
  for (idx i = 0; i < A.outerSize(); ++i) {
    for (spmatr::InnerIterator it(A, i); it; ++it) {
      size_t col = static_cast<size_t>(it.col());
      size_t idx = static_cast<size_t>(outer_index_[col] + col_nnz[col]);
      values_[idx] = it.value();
      col_nnz[col]++;
    }
  }

  // print the matrix:
  for (size_t i = 0; i < static_cast<size_t>(cols_); ++i) {
    std::cout << "Column " << i << std::endl;
    std::cout << "Outer Index: " << outer_index_[i] << " " << outer_index_[i + 1] << std::endl;
    for (size_t j = static_cast<size_t>(outer_index_[i]);
         j < static_cast<size_t>(outer_index_[i + 1]); ++j) {
      std::cout << "(" << inner_index_[j] << ", " << i << "): " << values_[j] << std::endl;
    }
  }
}

}  // namespace ax::math
