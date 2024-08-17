#pragma once

#include <cstddef>
#include <tuple>

#include "ax/core/config.hpp"
#include "ax/core/macros.hpp"
namespace ax::utils {

namespace details {

template <typename T, size_t> using identity = T;

template <typename T, typename Helper> struct t_tuple_impl;

template <typename T, size_t... seq> struct t_tuple_impl<T, std::index_sequence<seq...>> {
  using type = std::tuple<identity<T, seq>...>;
};

template <typename T, size_t dim> using dup_tuple =
    typename t_tuple_impl<T, std::make_index_sequence<dim>>::type;

}  // namespace details

/****************************** Tuple with Duplicated Type ******************************/

// @brief Alias for tuple of duplicated types.
template <typename T, size_t dim> using DupTuple = details::dup_tuple<T, dim>;

template <size_t dim> using Index_tuple = DupTuple<Index, dim>;
template <size_t dim> using real_tuple = DupTuple<real, dim>;

template <typename T, typename... Args>
AX_FORCE_INLINE DupTuple<T, sizeof...(Args)> dup_tuple(Args... args) {
  return DupTuple<T, sizeof...(Args)>{std::forward<Args>(args)...};
}

template <typename... Args> AX_FORCE_INLINE auto ituple(Args... args) {
  return dup_tuple<Index>(std::forward<Args>(args)...);
}

template <typename... Args> AX_FORCE_INLINE auto rtuple(Args... args) {
  return dup_tuple<real>(std::forward<Args>(args)...);
}

}  // namespace ax::utils
