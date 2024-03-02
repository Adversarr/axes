#pragma once
#include "common.hpp"
#include "axes/utils/ranges.hpp"

namespace ax::math {

template <idx dim, typename Fn> math::fieldr<dim> make_field(veci<dim> const& nd, Fn&& fn) {
  math::fieldr<dim> f(nd.prod());
  idx id = 0;
  for (idx i = 0; i < nd[0]; ++i) {
    for (idx j = 0; j < nd[1]; ++j) {
      for (idx k = 0; k < nd[2]; ++k) {
        f.col(id++) = fn(i, j, k);
      }
    }
  }
  return f;
}

template <typename F> auto enumerate_field(F && field) {
  return utils::ranges::iter_enumerate(field.colwise().begin(), field.colwise().end());
}

}  // namespace ax::math