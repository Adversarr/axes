#pragma once
#include <iostream>

#include "axes/core/math/common.hpp" // IWYU pragma: export
#include "axes/core/utils/ndrange.hpp"
namespace axes {

/****************************************
 * NOTE: Foreach Indexer, it have:
 *  1. value_type: indicates the return value type
 *  2. operator(...): Physical Range -> Device Range
 *  3. operator[d]: Device Range -> Physical Range
 *  4. Iterate(): To corresponding Iterator.
 ****************************************/
template <size_t dim> class NdRangeIndexer;

/**
 * @brief Nd Range Indexer.
 *
 * @param this_dim
 * @param arg
 */
template <size_t dim> class NdRangeIndexer {
public:
  std::array<size_t, dim> multipliers_;
  std::array<size_t, dim> shape_;

  /**
   * @brief Construct a new nd range indexer
   *
   * @param arr
   */
  constexpr explicit NdRangeIndexer(std::array<size_t, dim> arr) noexcept
      : shape_(arr) {
    multipliers_[dim - 1] = 1;
    for (size_t i = dim - 1; i > 0; --i) {
      multipliers_[i - 1] = multipliers_[i] * shape_[i];
    }
  }

  /**
   * @brief Construct a new nd-range indexer from (...)
   *
   * @tparam Args
   * @param arguments
   * @return
   */
  template <typename... Args>
  constexpr explicit NdRangeIndexer(Args... arguments) noexcept
      : NdRangeIndexer(
          std::array<size_t, dim>{static_cast<size_t>(arguments)...}) {}

  /**
   * @brief Default constructor for ndrangeindexer.
   *
   * @tparam Args
   * @return
   */
  template <typename... Args> constexpr explicit NdRangeIndexer() noexcept {
    std::fill(multipliers_.begin(), multipliers_.end(), 0);
    std::fill(shape_.begin(), shape_.end(), 0);
  }

  constexpr bool IsValid(std::array<size_t, dim> indices) const noexcept {
#pragma unroll 4
    for (size_t i = 0; i < dim; ++i) {
      if_unlikely(indices[i] >= shape_[i] || indices[i] < 0) { return false; }
    }
    return true;
  }

  template <typename... Args>
  constexpr size_t operator()(Args... indices) const noexcept {
    return operator()(std::array<size_t, dim>{static_cast<size_t>(indices)...});
  }

  constexpr size_t operator()(std::array<size_t, dim> indices) const noexcept {
    assert(IsValid(indices) && "Invalid indices.");
    size_t result = 0;
#pragma unroll 4
    for (size_t i = 0; i < dim; ++i) {
      result += indices[i] * multipliers_[i];
    }
    return result;
  }

  /**
   * @brief Get the shape of the RangeObject.
   *
   * @return
   */
  constexpr std::array<size_t, dim> Shape() const noexcept { return shape_; }

  /**
   * @brief Create an object that can iterate over.
   *
   * @return
   */
  constexpr auto Iterate() const noexcept { return NdRange<dim>(Shape()); }

  /**
   * @brief Returns the actual size of the range.
   *
   * @return
   */
  constexpr size_t Size() const noexcept { return multipliers_[0] * shape_[0]; }
};

template <> class NdRangeIndexer<1> {
public:
  inline size_t operator()(size_t id) const { return id; }

  constexpr bool IsValid(size_t this_size) const noexcept {
    return 0 <= this_size && this_size < this_dim_;
  }

  explicit constexpr NdRangeIndexer(size_t this_dim = 0)
      : this_dim_(this_dim) {}

  constexpr auto operator[](size_t id) const noexcept {
    return std::tuple<size_t>(id);
  }

  constexpr std::array<size_t, 1> Shape() const noexcept { return {this_dim_}; }

  constexpr size_t Size() const noexcept { return this_dim_; }

  constexpr auto Iterate() const noexcept {
    return NdRange<1>(std::array<size_t, 1>{this_dim_});
  }

protected:
  size_t this_dim_;
};

}  // namespace axes
