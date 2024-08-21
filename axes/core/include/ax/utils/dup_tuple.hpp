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

// @brief Alias for tuple of indices.
template <size_t dim> using IndexTuple = DupTuple<Index, dim>;
// @brief Alias for tuple of reals.
template <size_t dim> using RealTuple = DupTuple<Real, dim>;

template <typename T, typename... Args>
AX_FORCE_INLINE DupTuple<T, sizeof...(Args)> make_dup_tuple(Args&&... args) {
  return DupTuple<T, sizeof...(Args)>{std::forward<Args>(args)...};
}

template <typename... Args> AX_FORCE_INLINE auto make_index_tuple(Args&&... args) {
  return make_dup_tuple<Index>(std::forward<Args>(args)...);
}

template <typename... Args> AX_FORCE_INLINE auto make_real_tuple(Args&&... args) {
  return make_dup_tuple<Real>(std::forward<Args>(args)...);
}

}  // namespace ax::utils
