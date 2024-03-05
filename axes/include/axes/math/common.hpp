#pragma once

#include <Eigen/Dense>
#include <type_traits>

#include "axes/core/common.hpp"
#include "axes/utils/dup_tuple.hpp"

namespace ax::math {

/****************************** Vectors ******************************/

/**
 * @brief Vector type.
 *
 * @tparam Scalar The scalar type of the vector.
 * @tparam dim The dimension of the vector.
 */
template <typename Scalar, int dim> using vec = Eigen::Vector<Scalar, dim>;
template <typename Scalar, int dim> using rowvec = Eigen::RowVector<Scalar, dim>;

/****************************** Helper Class and Constants ******************************/
constexpr idx dynamic = Eigen::Dynamic;
template <typename T> using DB = Eigen::DenseBase<T>;
template <typename T> using DBr = DB<T> &;
template <typename T> using DBcr = DB<T> const &;
template <typename T> using MB = Eigen::MatrixBase<T>;
template <typename T> using MBr = MB<T> &;
template <typename T> using MBcr = MB<T> const &;
template <typename T> using AB = Eigen::ArrayBase<T>;
template <typename T> using ABr = AB<T> &;
template <typename T> using ABcr = AB<T> const &;

/**
 * @brief Most commonly used is `standard` realVector.
 *
 */
template <int dim> using vecr = vec<real, dim>;
template <int dim> using vecf = vec<float, dim>;
using vec2r = vec<real, 2>;        ///< Alias for 2D vector with double precision
                                   ///< floating point number.
using vec3r = vec<real, 3>;        ///< Alias for 3D vector with double precision
                                   ///< floating point number.
using vec4r = vec<real, 4>;        ///< Alias for 4D vector with double precision
                                   ///< floating point number.
using vecxr = vec<real, dynamic>;  ///< Alias for vector with double precision
                                   ///< floating point number.
using vec2f = vecf<2>;       ///< Alias for 2D vector with single precision
                                   ///< floating point number.
using vec3f = vecf<3>;       ///< Alias for 3D vector with single precision
                                   ///< floating point number.
using vec4f = vecf<4>;       ///< Alias for 4D vector with single precision
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

template <idx rows, idx cols> using matr = mat<real, rows, cols>;
template <idx rows, idx cols> using matf = mat<float, rows, cols>;
using matxxr = matr<dynamic, dynamic>;
using matxxf = matf<dynamic, dynamic>;

using mat2r = matr<2, 2>;
using mat3r = matr<3, 3>;
using mat4r = matr<4, 4>;
using mat2f = matf<2, 2>;
using mat3f = matf<3, 3>;
using mat4f = matf<4, 4>;

template <idx rows, idx cols> using mati = mat<idx, rows, cols>;
using matxxi = mati<dynamic, dynamic>;
using mat2i = mati<2, 2>;
using mat3i = mati<3, 3>;
using mat4i = mati<4, 4>;

using mat2xr = matr<2, dynamic>;
using mat3xr = matr<3, dynamic>;
using mat4xr = matr<4, dynamic>;
using matx2r = matr<dynamic, 2>;
using matx3r = matr<dynamic, 3>;
using matx4r = matr<dynamic, 4>;
using mat2xf = matf<2, dynamic>;
using mat3xf = matf<3, dynamic>;
using mat4xf = matf<4, dynamic>;
using matx2f = matf<dynamic, 2>;
using matx3f = matf<dynamic, 3>;
using matx4f = matf<dynamic, 4>;

using mat2xi = mati<2, dynamic>;
using mat3xi = mati<3, dynamic>;
using mat4xi = mati<4, dynamic>;
using matx2i = mati<dynamic, 2>;
using matx3i = mati<dynamic, 3>;
using matx4i = mati<dynamic, 4>;

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

/******************************************************************************
 * Matrix Meta data getter.
 *****************************************************************************/

using MatShape = std::pair<idx, idx>;

template <typename Derived>
 AX_CONSTEVAL idx rows_static(const Eigen::EigenBase<Derived> &) {
  return Derived::RowsAtCompileTime;
}

template <typename Derived>
 AX_CONSTEVAL idx cols_static(const Eigen::EigenBase<Derived> &) {
  return Derived::ColsAtCompileTime;
}

template <typename Derived>
 AX_CONSTEXPR idx rows(const Eigen::EigenBase<Derived> &mat) {
  return mat.rows();
}

template <typename Derived>
 AX_CONSTEXPR idx cols(const Eigen::EigenBase<Derived> &mat) {
  return mat.cols();
}

template <typename Derived>
AX_FORCE_INLINE MatShape shape_of(const Eigen::EigenBase<Derived> &mat) {
  return std::make_pair(rows(mat), cols(mat));
}

template <typename Derived>
AX_FORCE_INLINE MatShape shape_of_static(const Eigen::EigenBase<Derived> &mat) {
  return std::make_pair(rows_static(mat), cols_static(mat));
}

namespace details {
template <typename T, size_t... seq> AX_FORCE_INLINE vec<T, sizeof...(seq)> tuple_to_vector_impl(
    const utils::details::dup_tuple<T, sizeof...(seq)> &tuple, std::index_sequence<seq...>) {
  return vec<T, sizeof...(seq)>{std::get<seq>(tuple)...};
}

template <typename T, size_t ... seq> AX_FORCE_INLINE utils::DupTuple<T, sizeof...(seq)>
vector_to_tuple_impl(const vec<T, sizeof...(seq)> &vec, std::index_sequence<seq...>) {
  return utils::DupTuple<T, sizeof...(seq)>{vec[seq]...};
}

}  // namespace details

template <size_t dim> using index_tuple
    = utils::DupTuple<idx, dim>;  ///< Alias for tuple of duplicated indices.

/**
 * Convert a DupTuple to a vector.
 * @tparam T The type of the elements in the DupTuple.
 * @tparam dim The dimension of the DupTuple.
 * @param tuple The DupTuple to be converted.
 * @return A vector with the same elements as the DupTuple.
 */
template <typename T, size_t dim>
AX_FORCE_INLINE vec<T, dim> tuple_to_vector(const utils::DupTuple<T, dim> &tuple) {
  return details::tuple_to_vector_impl<T>(tuple, std::make_index_sequence<dim>());
}

/**
 * Convert a vector to a DupTuple.
 * @tparam T The type of the elements in the vector.
 * @tparam dim The dimension of the vector.
 * @param vec The vector to be converted.
 * @return A DupTuple with the same elements as the vector.
 */
template <typename T, size_t dim>
AX_FORCE_INLINE utils::DupTuple<T, dim> vector_to_tuple(const vec<T, dim> &vec) {
  return details::vector_to_tuple_impl<T>(vec, std::make_index_sequence<dim>());
}

/****************************** Constants ******************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/**
 * @brief The constant pi.
 */
template <typename Scalar = real> constexpr Scalar pi = static_cast<Scalar>(M_PI);
template <typename Scalar = real> constexpr Scalar pi_radian = static_cast<Scalar>(0.017453292519943);

/****************************** Common Ops ******************************/

/****************************** 1. ones ******************************/

template <idx rows, idx cols = 1, typename Scalar = real> AX_FORCE_INLINE auto ones() {
  return mat<Scalar, rows, cols>::Ones();
}

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto ones(idx cols) {
  return mat<Scalar, rows, dynamic>::Ones(rows, cols);
}

template <typename Scalar = real> AX_FORCE_INLINE auto ones(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Ones(rows, cols);
}

/****************************** 2. zeros ******************************/

template <idx rows, idx cols = 1, typename Scalar = real> AX_FORCE_INLINE auto zeros() {
  return mat<Scalar, rows, cols>::Zero();
}

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto zeros(idx cols) {
  return mat<Scalar, rows, dynamic>::Zero(rows, cols);
}

template <typename Scalar = real> AX_FORCE_INLINE auto zeros(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Zero(rows, cols);
}

/****************************** 3. constant ******************************/

template <idx rows, idx cols = 1, typename Scalar = real>
AX_FORCE_INLINE auto constant(Scalar value) {
  return mat<Scalar, rows, cols>::Constant(value);
}

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto constant(Scalar value, idx cols) {
  return mat<Scalar, rows, dynamic>::Constant(rows, cols, value);
}

template <typename Scalar = real> AX_FORCE_INLINE auto constant(Scalar value, idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Constant(rows, cols, value);
}

/****************************** 4. linspace ******************************/

template <idx rows, typename Scalar = real>
AX_FORCE_INLINE auto linspace(Scalar start, Scalar end) {
  return vec<Scalar, rows>::LinSpaced(rows, start, end);
}

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto linspace(Scalar end) {
  return linspace<rows, Scalar>(0, end);
}

template <typename Scalar = real>
AX_FORCE_INLINE auto linspace(Scalar start, Scalar end, idx rows) {
  return vec<Scalar, dynamic>::LinSpaced(rows, start, end);
}

/****************************** 5. arange ******************************/

template <typename Scalar = real> AX_FORCE_INLINE auto arange(idx stop) {
  return linspace<Scalar>(stop);
}

template <typename Scalar = real> AX_FORCE_INLINE auto arange(idx start, idx stop) {
  return linspace<Scalar>(start, stop, stop - start);
}

/****************************** 6. eye ******************************/

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto eye() {
  return mat<Scalar, rows, rows>::Identity();
}

template <typename Scalar = real> AX_FORCE_INLINE auto eye(idx rows) {
  return mat<Scalar, dynamic, dynamic>::Identity(rows, rows);
}

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto identity() {
  return eye<rows, Scalar>();
}

template <typename Scalar = real> AX_FORCE_INLINE auto identity(idx rows) {
  return eye<Scalar>(rows);
}

/****************************** 7. diag ******************************/

template <typename Derived> AX_FORCE_INLINE auto diag(
    MBcr<Derived> mat,
    char (*)[Derived::ColsAtCompileTime == 1 && Derived::RowsAtCompileTime != 1] = nullptr) {
  return mat.asDiagonal();
}

template <typename Derived> AX_FORCE_INLINE auto diag(
    MBcr<Derived> mat,
    char (*)[Derived::RowsAtCompileTime == 1 && Derived::ColsAtCompileTime != 1] = nullptr) {
  return diag(mat.transpose());
}

template <typename Derived>
AX_FORCE_INLINE auto diag(MBcr<Derived> mat,
                          char (*)[Derived::RowsAtCompileTime == Derived::ColsAtCompileTime]
                          = nullptr) {
  return mat.diagonal();
}

/****************************** 8. empty ******************************/

template <idx rows, idx cols = 1, typename Scalar = real> AX_FORCE_INLINE auto empty() {
  return mat<Scalar, rows, cols>{};
}

template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto empty(idx cols) {
  return mat<Scalar, rows, dynamic>{rows, cols};
}

template <typename Scalar = real> AX_FORCE_INLINE auto empty(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>{rows, cols};
}

/****************************** 9. from buffer ******************************/
// TODO:

/****************************** 10. unit ******************************/
template <idx dim, typename Scalar = real> AX_FORCE_INLINE auto unit(idx i) {
  return vec<Scalar, dim>::Unit(i);
}

/****************************** Iter methods ******************************/

template <typename Derived>
AX_FORCE_INLINE auto each(DBcr<Derived> mat, char (*)[Derived::ColsAtCompileTime != 1] = nullptr) {
  return mat.colwise();
}

template <typename Derived>
AX_FORCE_INLINE auto each(DBr<Derived> mat, char (*)[Derived::ColsAtCompileTime != 1] = nullptr) {
  return mat.colwise();
}

template <typename Derived>
AX_FORCE_INLINE decltype(auto) each(DBcr<Derived> mat,
                                    char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat;
}

template <typename Derived>
AX_FORCE_INLINE decltype(auto) each(DBr<Derived> mat,
                                    char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat;
}

/****************************** eval ******************************/
template <typename Derived> AX_FORCE_INLINE auto eval(DBcr<Derived> mat) { return mat.eval(); }

/****************************** cast ******************************/

template <typename To, typename Derived> AX_FORCE_INLINE auto cast(DBcr<Derived> mat) {
  return mat.template cast<To>();
}

template <typename To, typename From, typename = std::enable_if_t<std::is_arithmetic_v<From>>>
To cast(From value) {
  return static_cast<To>(value);
}

template <typename Derived> AX_FORCE_INLINE auto as_array(MBcr<Derived> mat) noexcept {
  return mat.array();
}

template <typename Derived> AX_FORCE_INLINE auto as_array(ABcr<Derived> arr) noexcept {
  return arr;
}

template <typename Derived> AX_FORCE_INLINE auto as_array(MBr<Derived> mat) noexcept {
  return mat.array();
}

template <typename Derived> AX_FORCE_INLINE auto as_array(ABr<Derived> arr) noexcept { return arr; }

template <typename Derived> AX_FORCE_INLINE auto as_matrix(ABcr<Derived> arr) noexcept {
  return arr.matrix();
}

template <typename Derived> AX_FORCE_INLINE auto as_matrix(MBcr<Derived> mat) noexcept {
  return mat;
}

template <typename Derived> AX_FORCE_INLINE auto as_matrix(ABr<Derived> arr) noexcept {
  return arr.matrix();
}

template <typename Derived> AX_FORCE_INLINE auto as_matrix(MBr<Derived> mat) noexcept {
  return mat;
}

/****************************** transpose ******************************/

template <typename Derived> AX_FORCE_INLINE auto transpose(DBcr<Derived> mat) noexcept {
  return mat.transpose();
}

template <typename Derived> AX_FORCE_INLINE auto transpose(DBr<Derived> mat) noexcept {
  return mat.transpose();
}

template <typename Derived> AX_FORCE_INLINE void transpose_(DBr<Derived> mat) noexcept {
  mat.transposeInPlace();
}

/****************************** reshape ******************************/

template <typename Derived> AX_FORCE_INLINE auto flatten(MBcr<Derived> mat) noexcept {
  return Eigen::Reshaped<const Derived, Derived::SizeAtCompileTime, 1>(mat.derived());
}

template <typename Derived> AX_FORCE_INLINE auto flatten(MBr<Derived> mat) noexcept {
  // TODO: test.
  return Eigen::Reshaped<const Derived, Derived::SizeAtCompileTime, 1>(mat.derived());
}

/****************************** field creation ******************************/

template <typename Derived> AX_FORCE_INLINE auto create_field(idx dofs) {
  return field<typename Derived::Scalar, Derived::RowsAtCompileTime>{Derived::RowsAtCompileTime,
                                                                     dofs};
}

// TODO: More create methods.

}  // namespace ax::math
