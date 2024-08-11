#pragma once

#include "common.hpp"  // IWYU pragma: export
#include <type_traits>
namespace ax::math {

/****************************** Scalar Type For ******************************/

template <typename T> struct scalar_of {
  using type = typename T::Scalar;  // For Eigen type.
};

template <> struct scalar_of<f32> {
  using type = f32;
};

template <> struct scalar_of<f64> {
  using type = f64;
};

/****************************** floating check ******************************/
template <typename F> constexpr bool is_scalar_v = std::is_floating_point_v<std::decay_t<F>>;
template <typename F> using enable_if_scalar_t = std::enable_if_t<is_scalar_v<F>, F>;

/****************************** convertion check ******************************/

namespace details {
template <typename Derived, template <typename> typename Base> constexpr bool is_eigen_convertible_v
    = std::is_convertible_v<std::decay_t<Derived>, Base<std::decay_t<Derived>>>;
}

template <typename F> constexpr bool is_eigen_v
    = details::is_eigen_convertible_v<F, Eigen::EigenBase>;

template <typename F> constexpr bool is_matrix_v
    = details::is_eigen_convertible_v<F, Eigen::MatrixBase>;

template <typename F> constexpr bool is_dense_v
    = details::is_eigen_convertible_v<F, Eigen::DenseBase>;

/****************************** compile time variable traits ******************************/
namespace details {
template <typename Derived, typename> struct EigenInfo;

template <typename Derived, typename = std::enable_if_t<is_eigen_v<Derived>>> struct EigenInfo {
  static constexpr idx rows = Derived::RowsAtCompileTime;
  static constexpr idx cols = Derived::ColsAtCompileTime;
  static constexpr idx size = Derived::SizeAtCompileTime;
};
}  // namespace details

template <typename Derived> constexpr idx rows_v = details::EigenInfo<Derived>::rows;
template <typename Derived> constexpr idx cols_v = details::EigenInfo<Derived>::cols;

template <typename Derived> constexpr bool is_vector_v = (cols_v<Derived> == 1);
template <typename Derived> constexpr bool is_row_vector_v = (rows_v<Derived> == 1);


template <typename A, typename B> constexpr bool same_rows_v = rows_v<A> == rows_v<B>;
template <typename A, typename B> constexpr bool same_cols_v = cols_v<A> == cols_v<B>;
template <typename A, typename B> constexpr bool same_shape_v = same_rows_v<A, B> && same_cols_v<A, B>;

}  // namespace ax::math
