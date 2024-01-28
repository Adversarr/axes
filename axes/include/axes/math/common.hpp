#pragma once

#include <Eigen/Dense>
#include <type_traits>

#include "axes/core/common.hpp"
#include "axes/utils/dup_tuple.hpp"

namespace axes {

/****************************** Vectors ******************************/

/**
 * @brief Vector type.
 *
 * @tparam Scalar The scalar type of the vector.
 * @tparam dim The dimension of the vector.
 */
template <typename Scalar, int dim> using vec = Eigen::Vector<Scalar, dim>;
template <typename Scalar, int dim> using rowvec = Eigen::RowVector<Scalar, dim>;

constexpr idx dynamic = Eigen::Dynamic;

/**
 * @brief Most commonly used is `standard` realVector.
 *
 */
template <int dim> using vecr = vec<real, dim>;
using vec2r = vec<real, 2>;        ///< Alias for 2D vector with double precision
                                   ///< floating point number.
using vec3r = vec<real, 3>;        ///< Alias for 3D vector with double precision
                                   ///< floating point number.
using vec4r = vec<real, 4>;        ///< Alias for 4D vector with double precision
                                   ///< floating point number.
using vecXr = vec<real, dynamic>;  ///< Alias for vector with double precision
                                   ///< floating point number.

/**
 * @brief idx vector type.
 *
 * @tparam dim The dimension of the idx vector.
 */
template <idx dim> using veci = vec<idx, dim>;

using vecxi = veci<dynamic>;
using vec2i = veci<2>;  ///< Alias for 2D idx vector.
using vec3i = veci<3>;  ///< Alias for 3D idx vector.
using vec4i = veci<4>;  ///< Alias for 4D idx vector.

/****************************** Matrices ******************************/

/**
 * @brief Default matrix type: Auto aligned, and if no template param is
 * provided, the default is dynamic size
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 */
template <typename Scalar, idx rows, idx cols> using mat = Eigen::Matrix<Scalar, rows, cols>;

///< Alias for matrix with double precision floating point number.
template <typename Scalar, idx rows, idx cols> using matr = mat<Scalar, rows, cols>;
using matxxr = mat<real, dynamic, dynamic>;

using mat2r = matr<real, 2, 2>;
using mat3r = matr<real, 3, 3>;
using mat4r = matr<real, 4, 4>;

using mat2xr = matr<real, 2, dynamic>;
using mat3xr = matr<real, 3, dynamic>;
using mat4xr = matr<real, 4, dynamic>;

/****************************** Field ******************************/

/**
 * @brief Field is the most important type in axes.
 */
template <typename Scalar, idx dim> using field = mat<Scalar, dim, dynamic>;

template <idx dim> using fieldr = field<real, dim>;
template <idx dim> using fieldi = field<idx, dim>;

using field1r = fieldr<1>;
using field2r = fieldr<2>;
using field3r = fieldr<3>;
using field4r = fieldr<4>;

using field1i = fieldi<1>;
using field2i = fieldi<2>;
using field3i = fieldi<3>;
using field4i = fieldi<4>;

/****************************** Scalar Type For ******************************/

template <typename T> struct scalar_of {
  using type = T::Scalar;  // For Eigen type.
};

template <> struct scalar_of<f32> {
  using type = f32;
};

template <> struct scalar_of<f64> {
  using type = f64;
};

/**
 * Meta programming utilities for matrices
 */
namespace math {

/******************************************************************************
 * Matrix Meta data getter.
 *****************************************************************************/

using MatShape = std::pair<idx, idx>;

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEVAL idx rows_static(const Eigen::EigenBase<Derived> &) {
  return Derived::RowsAtCompileTime;
}

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEVAL idx cols_static(const Eigen::EigenBase<Derived> &) {
  return Derived::ColsAtCompileTime;
}

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEXPR idx rows(const Eigen::EigenBase<Derived> &mat) {
  return mat.rows();
}

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEXPR idx cols(const Eigen::EigenBase<Derived> &mat) {
  return mat.cols();
}

template <typename Derived>
AXES_FORCE_INLINE MatShape shape_of(const Eigen::EigenBase<Derived> &mat) {
  return std::make_pair(rows(mat), cols(mat));
}

template <typename Derived>
AXES_FORCE_INLINE MatShape shape_of_static(const Eigen::EigenBase<Derived> &mat) {
  return std::make_pair(rows_static(mat), cols_static(mat));
}

}  // namespace math

namespace details {
template <typename T, size_t... seq> AXES_FORCE_INLINE vec<T, sizeof...(seq)> tuple_to_vector_impl(
    const utils::details::dup_tuple<T, sizeof...(seq)> &tuple, std::index_sequence<seq...>) {
  return vec<T, sizeof...(seq)>{std::get<seq>(tuple)...};
}
}  // namespace details

template <size_t dim> using indexTuple
    = utils::DupTuple<idx, dim>;  ///< Alias for tuple of duplicated indices.

/**
 * Convert a DupTuple to a vector.
 * @tparam T The type of the elements in the DupTuple.
 * @tparam dim The dimension of the DupTuple.
 * @param tuple The DupTuple to be converted.
 * @return A vector with the same elements as the DupTuple.
 */
template <typename T, size_t dim>
AXES_FORCE_INLINE vec<T, dim> tuple_to_vector(const utils::DupTuple<T, dim> &tuple) {
  return details::tuple_to_vector_impl<T>(tuple, std::make_index_sequence<dim>());
}

namespace math {

/****************************** Constants ******************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/**
 * @brief The constant pi.
 */
template <typename Scalar = real> constexpr Scalar pi = static_cast<Scalar>(M_PI);

/****************************** Common Ops ******************************/

/****************************** 1. ones ******************************/

template <idx rows, idx cols = 1, typename Scalar = real> AXES_FORCE_INLINE auto ones() {
  return mat<Scalar, rows, cols>::Ones();
}

template <idx rows, typename Scalar = real> AXES_FORCE_INLINE auto ones(idx cols) {
  return mat<Scalar, rows, dynamic>::Ones(rows, cols);
}

template <typename Scalar = real> AXES_FORCE_INLINE auto ones(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Ones(rows, cols);
}

/****************************** 2. zeros ******************************/

template <idx rows, idx cols = 1, typename Scalar = real> AXES_FORCE_INLINE auto zeros() {
  return mat<Scalar, rows, cols>::Zero();
}

template <idx rows, typename Scalar = real> AXES_FORCE_INLINE auto zeros(idx cols) {
  return mat<Scalar, rows, dynamic>::Zero(rows, cols);
}

template <typename Scalar = real> AXES_FORCE_INLINE auto zeros(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Zero(rows, cols);
}

/****************************** 3. constant ******************************/

template <idx rows, idx cols = 1, typename Scalar = real>
AXES_FORCE_INLINE auto constant(Scalar value) {
  return mat<Scalar, rows, cols>::Constant(value);
}

template <idx rows, typename Scalar = real>
AXES_FORCE_INLINE auto constant(Scalar value, idx cols) {
  return mat<Scalar, rows, dynamic>::Constant(rows, cols, value);
}

template <typename Scalar = real>
AXES_FORCE_INLINE auto constant(Scalar value, idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Constant(rows, cols, value);
}

/****************************** 4. linspace ******************************/

template <idx rows, typename Scalar = real>
AXES_FORCE_INLINE auto linspace(Scalar start, Scalar end) {
  return vec<Scalar, rows>::LinSpaced(rows, start, end);
}

template <idx rows, typename Scalar = real> AXES_FORCE_INLINE auto linspace(Scalar end) {
  return linspace<rows, Scalar>(0, end);
}

template <typename Scalar = real>
AXES_FORCE_INLINE auto linspace(Scalar start, Scalar end, idx rows) {
  return vec<Scalar, dynamic>::LinSpaced(rows, start, end);
}

/****************************** 5. arange ******************************/

template <typename Scalar = real> AXES_FORCE_INLINE auto arange(idx stop) {
  return linspace<Scalar>(stop);
}

template <typename Scalar = real> AXES_FORCE_INLINE auto arange(idx start, idx stop) {
  return linspace<Scalar>(start, stop, stop - start);
}

/****************************** 6. eye ******************************/

template <idx rows, typename Scalar = real> AXES_FORCE_INLINE auto eye() {
  return mat<Scalar, rows, rows>::Identity();
}

template <typename Scalar = real> AXES_FORCE_INLINE auto eye(idx rows) {
  return mat<Scalar, dynamic, dynamic>::Identity(rows, rows);
}

template <idx rows, typename Scalar = real> AXES_FORCE_INLINE auto identity() {
  return eye<rows, Scalar>();
}

template <typename Scalar = real> AXES_FORCE_INLINE auto identity(idx rows) {
  return eye<Scalar>(rows);
}

/****************************** 7. diag ******************************/

template <typename Derived> AXES_FORCE_INLINE auto diag(
    const Eigen::MatrixBase<Derived> &mat,
    char (*)[Derived::ColsAtCompileTime == 1 && Derived::RowsAtCompileTime != 1] = nullptr) {
  return mat.asDiagonal();
}

template <typename Derived> AXES_FORCE_INLINE auto diag(
    const Eigen::MatrixBase<Derived> &mat,
    char (*)[Derived::RowsAtCompileTime == 1 && Derived::ColsAtCompileTime != 1] = nullptr) {
  return diag(mat.transpose());
}

template <typename Derived>
AXES_FORCE_INLINE auto diag(const Eigen::MatrixBase<Derived> &mat,
                            char (*)[Derived::RowsAtCompileTime == Derived::ColsAtCompileTime]
                            = nullptr) {
  return mat.diagonal();
}

/****************************** 8. empty ******************************/

template <idx rows, idx cols = 1, typename Scalar = real> AXES_FORCE_INLINE auto empty() {
  return mat<Scalar, rows, cols>{};
}

template <idx rows, typename Scalar = real> AXES_FORCE_INLINE auto empty(idx cols) {
  return mat<Scalar, rows, dynamic>{rows, cols};
}

template <typename Scalar = real> AXES_FORCE_INLINE auto empty(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>{rows, cols};
}

/****************************** 9. from buffer ******************************/
// TODO:

}  // namespace math

/****************************** Iter methods ******************************/

template <typename Derived>
AXES_FORCE_INLINE auto iter(const Eigen::DenseBase<Derived> &mat,
                            char (*)[Derived::ColsAtCompileTime != 1] = nullptr) {
  return mat.colwise();
}

template <typename Derived>
AXES_FORCE_INLINE auto iter(Eigen::DenseBase<Derived> &mat,
                            char (*)[Derived::ColsAtCompileTime != 1] = nullptr) {
  return mat.colwise();
}

template <typename Derived>
AXES_FORCE_INLINE decltype(auto) iter(const Eigen::DenseBase<Derived> &mat,
                                      char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat;
}

template <typename Derived>
AXES_FORCE_INLINE decltype(auto) iter(Eigen::DenseBase<Derived> &mat,
                                      char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat;
}

/****************************** eval ******************************/
template <typename Derived> AXES_FORCE_INLINE auto eval(const Eigen::DenseBase<Derived> &mat) {
  return mat.eval();
}

/****************************** field creation ******************************/

template <typename Derived> AXES_FORCE_INLINE auto create(idx dofs) {
  return field<typename Derived::Scalar, Derived::RowsAtCompileTime>{Derived::RowsAtCompileTime,
                                                                     dofs};
}

}  // namespace axes
