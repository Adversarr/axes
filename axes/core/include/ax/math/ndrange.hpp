#pragma once
#include "ax/math/common.hpp"
#include "ax/utils/ndrange.hpp"
#include "ax/math/shape.hpp"

namespace ax::math {

/****************************** iota ******************************/
namespace details {
template<typename Derived, size_t ... seq>
AX_FORCE_INLINE auto ndrange_from_vector_impl(math::DBcr<Derived> ends, std::index_sequence<seq...>) {
  return utils::ndrange<typename Derived::Scalar>(ends[seq]...);
}
}  // namespace details

template<int dim>
AX_FORCE_INLINE auto ndrange(math::IndexVector<dim> const& ends) {
  return details::ndrange_from_vector_impl(ends, std::make_index_sequence<dim>{});
}

}