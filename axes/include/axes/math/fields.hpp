#pragma once
#include "common.hpp"
#include "axes/utils/ranges.hpp"

namespace ax::math {

template <typename F> auto enumerate_field(F && field) {
  return utils::ranges::iter_enumerate(field.colwise().begin(), field.colwise().end());
}

template<typename Scalar, idx dim>
class FieldAccessor {
  field<Scalar, dim>& data_;
};

}  // namespace ax::math