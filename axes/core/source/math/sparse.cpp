#include "ax/math/sparse.hpp"
namespace ax::math {

RealSparseMatrix make_sparse_matrix(Index rows, Index cols, RealSparseCOO const& coeff_list) {
  RealSparseMatrix mat(rows, cols);
  mat.setFromTriplets(coeff_list.begin(), coeff_list.end());
  return mat;
}

RealSparseMatrix make_sparse_matrix(Index rows, Index cols, std::vector<SparseIndex> const& row,
                          std::vector<SparseIndex> const& col, std::vector<Real> const& val) {
  size_t const n = row.size();
  RealSparseCOO coeff_list;
  coeff_list.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    coeff_list.push_back({row[i], col[i], val[i]});
  }
  return make_sparse_matrix(rows, cols, coeff_list);
}
}  // namespace ax::math
