#pragma once
#include "axes/math/common.hpp"
#include "axes/utils/iota.hpp"
namespace ax::utils {

/****************************** iota ******************************/
namespace details {
template<typename Derived, size_t ... seq>
AX_FORCE_INLINE auto multi_iota_from_vector_impl(math::DBcr<Derived> ends, std::index_sequence<seq...>) {
  return multi_iota(ends[seq]...);
}
}  // namespace details

template<idx dim>
AX_FORCE_INLINE auto multi_iota(math::veci<dim> const& ends) {
  return details::multi_iota_from_vector_impl(ends, std::make_index_sequence<dim>{});
}

}