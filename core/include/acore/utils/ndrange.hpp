#pragma once

#include <array>

#include "acore/utils/common.hpp"

namespace axes {
namespace details {
template <size_t dim> constexpr auto get_zero_array() {
  std::array<size_t, dim> arr;
  for (auto &a : arr) {
    a = 0;
  }
  return arr;
}
}  // namespace details

/**
 * @brief Iterator for NdRange
 *
 * @return
 */
template <int dim> struct NdRangeIterator {
  using iterator_category = std::random_access_iterator_tag;
  using container_type = std::array<size_t, dim>;

  /**
   * @brief Get the value of cursor
   *
   * @return
   */
  constexpr decltype(auto) operator*() const noexcept { return dims_; }

  constexpr bool operator!=(const NdRangeIterator &another) const noexcept {
    return dims_ != another.dims_;
  }

  template <int d> inline void Advance() noexcept {
    size_t &cur_dim = dims_[d];
    ++cur_dim;
    if constexpr (d > 0) {
      if_unlikely(cur_dim >= ub_[d]) {
        cur_dim = 0;
        Advance<d - 1>();
      }
    }
  }

  NdRangeIterator<dim> &operator++() {
    Advance<dim - 1>();
    return *this;
  }

  /**
   * @brief Allow copy
   */
  constexpr NdRangeIterator(const NdRangeIterator &) = default;

  /**
   * @brief Construct a new NdRangeIterator, with current dim = dim,
   *  upper bound = ub.
   *
   * @param dims
   * @param ub
   */
  constexpr NdRangeIterator(const container_type &dims,
                            const container_type &ub) noexcept
      : dims_(dims), ub_(ub) {}
  container_type dims_;
  const container_type ub_;
};

/**
 * @brief NdRange object, can iterate over.
 *
 * @param dims
 */
template <int dim> struct NdRange {
  using container_type = std::array<size_t, dim>;

  // Same as 'Indexer'
  // 1. array input, i.e. container_type initialization
  constexpr explicit NdRange(container_type dims) : dims_{dims} {}

  // 2. An input sequence, that is able to construct a container_type.
  template <typename... Args,
            typename = std::enable_if_t<
                std::is_constructible_v<container_type, size_t, Args...>
                && !std::is_same_v<std::remove_cv_t<size_t>, container_type>>>
  constexpr explicit NdRange(size_t a, Args &&...args)
      : NdRange(container_type(a, args...)) {}

  // NOLINTBEGIN
  /**
   * @brief Returns iterator
   *
   * @return
   */
  constexpr auto begin() const noexcept {
    return NdRangeIterator<dim>(details::get_zero_array<dim>(), dims_);
  }

  /**
   * @brief Returns iterator.
   *
   * @return
   */
  constexpr auto end() const noexcept {
    auto arr = details::get_zero_array<dim>();
    arr[0] = dims_[0];
    return NdRangeIterator<dim>(arr, dims_);
  }
  // NOLINTEND

  const container_type dims_;
};

/**
 * Deduction guides for ndrange
 */
template <typename... Args> NdRange(std::array<size_t, sizeof...(Args)> &&dims)
    -> NdRange<sizeof...(Args)>;
template <typename... Args> NdRange(Args &&...dims) -> NdRange<sizeof...(Args)>;

/**
 * @brief Create a new NdRange object.
 *
 * @tparam Args 
 * @param args 
 * @return 
 */
template <typename... Args> constexpr auto make_range(Args &&...args) {
  return NdRange<sizeof...(Args)>{std::array<size_t, sizeof...(Args)>{
      static_cast<size_t>(std::forward<Args>(args))...}};
}

}  // namespace axes
