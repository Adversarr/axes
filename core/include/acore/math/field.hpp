#pragma once

#include <vector>

#include "acore/math/indexer.hpp"  // IWYU pragma: export
#include "common.hpp"

namespace axes {

template <typename Field, typename Indexer> class FieldView;
/**
 * @brief Field of data, only the internal data is contained.
 *
 * @tparam Data
 */
template <typename Tp, typename Allocator = std::allocator<Tp>> class Field {
public:
  using ValueType = Tp;

  template <typename Indexer = void> auto CreateView() {
    return FieldView<Field, Indexer>(*this);
  }

  /**
   * @brief Fit the field to the indexer by size.
   *
   * @tparam Indexer 
   * @param getter 
   * @return 
   */
  template <typename Indexer> Field& FitIndexer(Indexer&& getter) {
    internal_data_.resize(getter.Size());
  }

  
  /**
   * @brief Foreach field element, apply fn.
   *
   * @tparam Fn 
   * @param fn 
   * @return 
   */
  template <typename Fn, std::enable_if_t<std::is_function_v<Fn>>>
  Field& Foreach(Fn&& fn) {
    std::for_each(internal_data_.begin(), internal_data_.end(),
                  std::forward<Fn>(fn));
  }

  /**
   * @brief Foreach field element, apply fn. (const ver)
   *
   * @tparam Fn 
   * @param fn 
   * @return 
   */
  template <typename Fn, std::enable_if_t<std::is_function_v<Fn>>>
  Field& Foreach(Fn&& fn) const {
    std::for_each(internal_data_.cbegin(), internal_data_.cend(),
                  std::forward<Fn>(fn));
  }


  constexpr auto begin() noexcept { return internal_data_.begin(); }
  constexpr auto end() noexcept { return internal_data_.end(); }
  constexpr auto begin() const noexcept { return internal_data_.begin(); }
  constexpr auto end() const noexcept { return internal_data_.end(); }

  // With boundary check
  decltype(auto) At(size_t n) const { return internal_data_.at(n); }

  decltype(auto) At(size_t n) { return internal_data_.at(n); }

  // Without boundary check
  decltype(auto) operator[](size_t n) const noexcept {
    return internal_data_[n];
  }
  decltype(auto) operator[](size_t n) noexcept { return internal_data_[n]; }

  size_t Size() const noexcept { return internal_data_.size(); }

  Field& Resize(size_t n) {
    internal_data_.resize(n);
    return *this;
  }

private:
  std::vector<Tp, Allocator> internal_data_;
};

template <typename Scalar, int dim> using VectorField
    = Field<axes::Vector<Scalar, dim>>;

/**
 * @brief Iterator for FieldView object.
 *
 * @tparam Field
 * @tparam Indexer
 */
template <typename Field, typename Indexer> class FieldViewIterator {
public:
private:
  Field& field_;
  Indexer getter_;
};

template <typename Field> class FieldViewIterator<Field, void> {
public:
  using FieldValue = typename std::remove_const_t<Field>::ValueType;

  FieldViewIterator(Field& field, size_t cursor)
      : field_(field), cursor_(cursor) {}

  std::pair<size_t, FieldValue&> operator*() {
    return std::pair<size_t, FieldValue&>(cursor_, field_.At(cursor_));
  }

  std::pair<size_t, const FieldValue&> operator*() const {
    return std::pair<size_t, const FieldValue&>(cursor_, field_.At(cursor_));
  }

  decltype(auto) operator++() {
    ++cursor_;
    return *this;
  }

  bool operator!=(FieldViewIterator rhs) const noexcept {
    return cursor_ != rhs.cursor_;
  }
  bool operator==(FieldViewIterator rhs) const noexcept {
    return cursor_ == rhs.cursor_;
  }

private:
  Field& field_;
  size_t cursor_{0};
};
template <typename Field, typename Indexer> class FieldView {};

// Use Indexer == void for those 1D arrays.
template <typename Field> class FieldView<Field, void> {
public:
  FieldView(Field& field) : field_(field) {}

  auto begin() noexcept { return FieldViewIterator<Field, void>(field_, 0); };

  auto end() noexcept {
    return FieldViewIterator<Field, void>(field_, field_.Size());
  };

private:
  Field& field_;
};

}  // namespace axes
