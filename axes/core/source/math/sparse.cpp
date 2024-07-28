#include "ax/math/sparse.hpp"
namespace ax::math {

spmatr make_sparse_matrix(idx rows, idx cols, sp_coeff_list const& coeff_list) {
  spmatr mat(rows, cols);
  mat.setFromTriplets(coeff_list.begin(), coeff_list.end());
  return mat;
}

spmatr make_sparse_matrix(idx rows, idx cols, std::vector<idx> const& row,
                          std::vector<idx> const& col, std::vector<real> const& val) {
  size_t const n = row.size();
  sp_coeff_list coeff_list;
  coeff_list.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    coeff_list.push_back({row[i], col[i], val[i]});
  }
  return make_sparse_matrix(rows, cols, coeff_list);
}
}  // namespace ax::math
