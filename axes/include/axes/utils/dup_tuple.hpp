#pragma once

#include <cstddef>
#include <tuple>

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

/// @brief Alias for tuple of duplicated types.
template <typename T, size_t dim> using DupTuple = details::dup_tuple<T, dim>;
}  // namespace axes::utils
