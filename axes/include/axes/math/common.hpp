/**
 * @file common.hpp
 * @brief Common math utilities and types for axes library.
 */
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

/**
 * @brief Constant representing dynamic size.
 */
constexpr idx dynamic = Eigen::Dynamic;

/**
 * @brief Helper type aliases for Eigen types.
 */
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
 */
template <int dim> using vecr = vec<real, dim>;
template <int dim> using vecf = vec<float, dim>;
using vec2r = vec<real, 2>;  ///< Alias for 2D vector with double precision floating point number.
using vec3r = vec<real, 3>;  ///< Alias for 3D vector with double precision floating point number.
using vec4r = vec<real, 4>;  ///< Alias for 4D vector with double precision floating point number.
using vecxr
    = vec<real, dynamic>;  ///< Alias for vector with double precision floating point number.
using vec2f = vecf<2>;     ///< Alias for 2D vector with single precision floating point number.
using vec3f = vecf<3>;     ///< Alias for 3D vector with single precision floating point number.
using vec4f = vecf<4>;     ///< Alias for 4D vector with single precision floating point number.

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
 * @brief Default matrix type: Auto aligned, and if no template param is provided, the default is
 * dynamic size.
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
using fieldxr = fieldr<dynamic>;

using field1i = fieldi<1>;
using field2i = fieldi<2>;
using field3i = fieldi<3>;
using field4i = fieldi<4>;
using fieldxi = fieldi<dynamic>;

/******************************************************************************
 * Matrix Meta data getter.
 *****************************************************************************/

/**
 * @brief Alias for the shape of a matrix.
 */
using MatShape = std::pair<idx, idx>;

/**
 * @brief Get the number of rows of a matrix at compile time.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of rows of the matrix.
 */
template <typename Derived> AX_CONSTEVAL idx rows_static(const Eigen::EigenBase<Derived> &) {
  return Derived::RowsAtCompileTime;
}

/**
 * @brief Get the number of columns of a matrix at compile time.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of columns of the matrix.
 */
template <typename Derived> AX_CONSTEVAL idx cols_static(const Eigen::EigenBase<Derived> &) {
  return Derived::ColsAtCompileTime;
}

/**
 * @brief Get the number of rows of a matrix at runtime.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of rows of the matrix.
 */
template <typename Derived> AX_CONSTEXPR idx rows(const Eigen::EigenBase<Derived> &mat) {
  return mat.rows();
}

/**
 * @brief Get the number of columns of a matrix at runtime.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of columns of the matrix.
 */
template <typename Derived> AX_CONSTEXPR idx cols(const Eigen::EigenBase<Derived> &mat) {
  return mat.cols();
}

/**
 * @brief Get the shape of a matrix at runtime.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The shape of the matrix as a pair of (rows, columns).
 */
template <typename Derived>
AX_FORCE_INLINE MatShape shape_of(const Eigen::EigenBase<Derived> &mat) {
  return std::make_pair(rows(mat), cols(mat));
}

/**
 * @brief Get the shape of a matrix at compile time.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The shape of the matrix as a pair of (rows, columns).
 */
template <typename Derived>
AX_FORCE_INLINE MatShape shape_of_static(const Eigen::EigenBase<Derived> &mat) {
  return std::make_pair(rows_static(mat), cols_static(mat));
}

namespace details {
/**
 * @brief Convert a tuple to a vector.
 *
 * @tparam T The type of the elements in the tuple.
 * @tparam seq The sequence of indices.
 * @param tuple The tuple to be converted.
 * @return A vector with the same elements as the tuple.
 */
template <typename T, size_t... seq> AX_FORCE_INLINE vec<T, sizeof...(seq)> tuple_to_vector_impl(
    const utils::details::dup_tuple<T, sizeof...(seq)> &tuple, std::index_sequence<seq...>) {
  return vec<T, sizeof...(seq)>{std::get<seq>(tuple)...};
}

/**
 * @brief Convert a vector to a tuple.
 *
 * @tparam T The type of the elements in the vector.
 * @tparam seq The sequence of indices.
 * @param vec The vector to be converted.
 * @return A tuple with the same elements as the vector.
 */
template <typename T, size_t... seq>
AX_FORCE_INLINE utils::DupTuple<T, sizeof...(seq)> vector_to_tuple_impl(
    const vec<T, sizeof...(seq)> &vec, std::index_sequence<seq...>) {
  return utils::DupTuple<T, sizeof...(seq)>{vec[seq]...};
}

}  // namespace details

/**
 * @brief Convert a DupTuple to a vector.
 *
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
 * @brief Convert a vector to a DupTuple.
 *
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

/**
 * @brief The constant pi in radians.
 */
template <typename Scalar = real> constexpr Scalar pi_radian
    = static_cast<Scalar>(0.017453292519943);

/**
 * @brief The constant NaN (Not-a-Number).
 */
template <typename Scalar = real> constexpr Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();

/**
 * @brief The constant infinity.
 */
template <typename Scalar = real> constexpr Scalar inf = std::numeric_limits<Scalar>::infinity();

/****************************** Common Ops ******************************/

/****************************** 1. ones ******************************/

/**
 * @brief Create a matrix filled with ones.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @return A matrix filled with ones.
 */
template <idx rows, idx cols = 1, typename Scalar = real> AX_FORCE_INLINE auto ones() {
  return mat<Scalar, rows, cols>::Ones();
}

/**
 * @brief Create a matrix filled with ones.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with ones.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto ones(idx cols) {
  return mat<Scalar, rows, dynamic>::Ones(rows, cols);
}

/**
 * @brief Create a matrix filled with ones.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with ones.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto ones(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Ones(rows, cols);
}

/**
 * @brief Fill a matrix with ones.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam r The number of rows of the matrix.
 * @tparam c The number of columns of the matrix.
 * @param mat The matrix to be filled.
 */
template <typename Scalar, idx r, idx c> AX_FORCE_INLINE void ones_(const mat<Scalar, r, c> &mat) {
  mat.setOnes();
}

/**
 * @brief Fill a scalar with ones.
 *
 * @tparam Scalar The scalar type.
 * @param mat The scalar to be filled.
 */
template <typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
AX_FORCE_INLINE void ones_(const Scalar &mat) {
  mat.setOnes();
}

/**
 * @brief Create a ones value.
 *
 * @tparam T The type of the value.
 * @return A ones value.
 */
template <typename T> AX_FORCE_INLINE T make_ones() {
  if constexpr (std::is_arithmetic_v<T>) {
    return T(1);
  } else {
    return T::Ones();
  }
}

/****************************** 2. zeros ******************************/

/**
 * @brief Create a matrix filled with zeros.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @return A matrix filled with zeros.
 */
template <idx rows, idx cols = 1, typename Scalar = real> AX_FORCE_INLINE auto zeros() {
  return mat<Scalar, rows, cols>::Zero();
}

/**
 * @brief Create a matrix filled with zeros.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with zeros.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto zeros(idx cols) {
  return mat<Scalar, rows, dynamic>::Zero(rows, cols);
}

/**
 * @brief Create a matrix filled with zeros.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with zeros.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto zeros(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Zero(rows, cols);
}

/**
 * @brief Fill a matrix with zeros.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam r The number of rows of the matrix.
 * @tparam c The number of columns of the matrix.
 * @param mat The matrix to be filled.
 */
template <typename Scalar, idx r, idx c> AX_FORCE_INLINE void zeros_(const mat<Scalar, r, c> &mat) {
  mat.setZero();
}

/**
 * @brief Fill a scalar with zeros.
 *
 * @tparam Scalar The scalar type.
 * @param mat The scalar to be filled.
 */
template <typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
AX_FORCE_INLINE void zeros_(const Scalar &mat) {
  mat.setZero();
}

/**
 * @brief Create a zeros value.
 *
 * @tparam T The type of the value.
 * @return A zeros value.
 */
template <typename T> AX_FORCE_INLINE T make_zeros() {
  if constexpr (std::is_arithmetic_v<T>) {
    return T(0);
  } else {
    return T::Zero();
  }
}

/****************************** 3. constant ******************************/

/**
 * @brief Create a matrix filled with a constant value.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param value The constant value.
 * @return A matrix filled with the constant value.
 */
template <idx rows, idx cols = 1, typename Scalar = real>
AX_FORCE_INLINE auto constant(Scalar value) {
  return mat<Scalar, rows, cols>::Constant(value);
}

/**
 * @brief Create a matrix filled with a constant value.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param value The constant value.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with the constant value.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto constant(Scalar value, idx cols) {
  return mat<Scalar, rows, dynamic>::Constant(rows, cols, value);
}

/**
 * @brief Create a matrix filled with a constant value.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param value The constant value.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with the constant value.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto constant(Scalar value, idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>::Constant(rows, cols, value);
}

/**
 * @brief Fill a matrix with a constant value.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam r The number of rows of the matrix.
 * @tparam c The number of columns of the matrix.
 * @param mat The matrix to be filled.
 * @param value The constant value.
 */
template <typename Scalar, idx r, idx c>
AX_FORCE_INLINE void constant_(const mat<Scalar, r, c> &mat, Scalar value) {
  mat.setConstant(value);
}

/**
 * @brief Fill a scalar with a constant value.
 *
 * @tparam Scalar The scalar type.
 * @param mat The scalar to be filled.
 * @param value The constant value.
 */
template <typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
AX_FORCE_INLINE void constant_(const Scalar &mat, Scalar value) {
  mat.setConstant(value);
}

/**
 * @brief Create a constant value.
 *
 * @tparam T The type of the value.
 * @param value The constant value.
 * @return A constant value.
 */
template <typename T, typename Scalar> AX_FORCE_INLINE T make_constant(Scalar value) {
  if constexpr (std::is_arithmetic_v<T>) {
    return T(value);
  } else {
    return T::Constant(value);
  }
}

/****************************** 4. linspace ******************************/

/**
 * @brief Create a vector filled with linearly spaced values.
 *
 * @tparam rows The number of rows of the vector.
 * @tparam Scalar The scalar type of the vector.
 * @param start The start value.
 * @param end The end value.
 * @return A vector filled with linearly spaced values.
 */
template <idx rows, typename Scalar = real>
AX_FORCE_INLINE auto linspace(Scalar start, Scalar end) {
  return vec<Scalar, rows>::LinSpaced(rows, start, end);
}

/**
 * @brief Create a vector filled with linearly spaced values.
 *
 * @tparam Scalar The scalar type of the vector.
 * @param end The end value.
 * @return A vector filled with linearly spaced values.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto linspace(Scalar end) {
  return linspace<rows, Scalar>(0, end);
}

/**
 * @brief Create a vector filled with linearly spaced values.
 *
 * @tparam Scalar The scalar type of the vector.
 * @param start The start value.
 * @param end The end value.
 * @param rows The number of rows of the vector.
 * @return A vector filled with linearly spaced values.
 */
template <typename Scalar = real>
AX_FORCE_INLINE auto linspace(Scalar start, Scalar end, idx rows) {
  return vec<Scalar, dynamic>::LinSpaced(rows, start, end);
}

/****************************** 5. arange ******************************/

/**
 * @brief Create a vector filled with values from start to stop.
 *
 * @tparam Scalar The scalar type of the vector.
 * @param stop The stop value.
 * @return A vector filled with values from start to stop.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto arange(idx stop) {
  return linspace<Scalar>(stop);
}

/**
 * @brief Create a vector filled with values from start to stop.
 *
 * @tparam Scalar The scalar type of the vector.
 * @param start The start value.
 * @param stop The stop value.
 * @return A vector filled with values from start to stop.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto arange(idx start, idx stop) {
  return linspace<Scalar>(start, stop, stop - start);
}

/****************************** 6. eye ******************************/

/**
 * @brief Create an identity matrix.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @return An identity matrix.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto eye() {
  return mat<Scalar, rows, rows>::Identity();
}

/**
 * @brief Create an identity matrix.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An identity matrix.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto eye(idx rows) {
  return mat<Scalar, dynamic, dynamic>::Identity(rows, rows);
}

/**
 * @brief Create an identity matrix.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An identity matrix.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto identity() {
  return eye<rows, Scalar>();
}

/**
 * @brief Create an identity matrix.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An identity matrix.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto identity(idx rows) {
  return eye<Scalar>(rows);
}

/****************************** 7. diag ******************************/

/**
 * @brief Create a diagonal matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be converted to a diagonal matrix.
 * @return A diagonal matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto diag(
    MBcr<Derived> mat,
    char (*)[Derived::ColsAtCompileTime == 1 && Derived::RowsAtCompileTime != 1] = nullptr) {
  return mat.asDiagonal();
}

/**
 * @brief Create a diagonal matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be converted to a diagonal matrix.
 * @return A diagonal matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto diag(
    MBcr<Derived> mat,
    char (*)[Derived::RowsAtCompileTime == 1 && Derived::ColsAtCompileTime != 1] = nullptr) {
  return diag(mat.transpose());
}

/**
 * @brief Create a diagonal matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be converted to a diagonal matrix.
 * @return A diagonal matrix.
 */
template <typename Derived>
AX_FORCE_INLINE auto diag(MBcr<Derived> mat,
                          char (*)[Derived::RowsAtCompileTime == Derived::ColsAtCompileTime]
                          = nullptr) {
  return mat.diagonal();
}

/****************************** 8. empty ******************************/

/**
 * @brief Create an empty matrix.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @return An empty matrix.
 */
template <idx rows, idx cols = 1, typename Scalar = real> AX_FORCE_INLINE auto empty() {
  return mat<Scalar, rows, cols>{};
}

/**
 * @brief Create an empty matrix.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An empty matrix.
 */
template <idx rows, typename Scalar = real> AX_FORCE_INLINE auto empty(idx cols) {
  return mat<Scalar, rows, dynamic>{rows, cols};
}

/**
 * @brief Create an empty matrix.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An empty matrix.
 */
template <typename Scalar = real> AX_FORCE_INLINE auto empty(idx rows, idx cols) {
  return mat<Scalar, dynamic, dynamic>{rows, cols};
}

/****************************** 9. from buffer ******************************/
// TODO:

/****************************** 10. unit ******************************/

/**
 * @brief Create a unit vector.
 *
 * @tparam dim The dimension of the vector.
 * @tparam Scalar The scalar type of the vector.
 * @param i The index of the unit vector.
 * @return A unit vector.
 */
template <idx dim, typename Scalar = real> AX_FORCE_INLINE auto unit(idx i) {
  return vec<Scalar, dim>::Unit(i);
}

/****************************** Iter methods ******************************/

/**
 * @brief Iterate over the elements of a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be iterated over.
 * @param func The function to be applied to each element.
 */
template <typename Derived>
AX_FORCE_INLINE auto each(DBcr<Derived> mat, char (*)[Derived::ColsAtCompileTime != 1] = nullptr) {
  return mat.colwise();
}

/**
 * @brief Iterate over the elements of a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be iterated over.
 * @param func The function to be applied to each element.
 */
template <typename Derived>
AX_FORCE_INLINE auto each(DBr<Derived> mat, char (*)[Derived::ColsAtCompileTime != 1] = nullptr) {
  return mat.colwise();
}

/**
 * @brief Iterate over the elements of a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be iterated over.
 * @param func The function to be applied to each element.
 */
template <typename Derived>
AX_FORCE_INLINE decltype(auto) each(DBcr<Derived> mat,
                                    char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat;
}

/**
 * @brief Iterate over the elements of a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be iterated over.
 * @param func The function to be applied to each element.
 */
template <typename Derived>
AX_FORCE_INLINE decltype(auto) each(DBr<Derived> mat,
                                    char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat;
}

/****************************** eval ******************************/

/**
 * @brief Evaluate a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be evaluated.
 * @return The evaluated matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto eval(DBcr<Derived> mat) { return mat.eval(); }

/****************************** cast ******************************/

/**
 * @brief Cast a matrix to a different type.
 *
 * @tparam To The type to cast to.
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be cast.
 * @return The casted matrix.
 */
template <typename To, typename Derived> AX_FORCE_INLINE auto cast(DBcr<Derived> mat) {
  return mat.template cast<To>();
}

/**
 * @brief Cast a matrix to a different type.
 *
 * @tparam To The type to cast to.
 * @tparam From The type to cast from.
 * @return The casted value
 */
template <typename To, typename From, typename = std::enable_if_t<std::is_arithmetic_v<From>>>
To cast(From value) {
  return static_cast<To>(value);
}

/****************************** as_array ******************************/

/**
 * @brief Convert a matrix to an array.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be converted.
 * @return The array.
 */
template <typename Derived> AX_FORCE_INLINE auto as_array(MBcr<Derived> mat) noexcept {
  return mat.array();
}

template <typename Derived> AX_FORCE_INLINE auto as_array(ABcr<Derived> arr) noexcept {
  return arr;
}

/**
 * @brief Convert an array to a matrix.
 *
 * @tparam Derived The derived type of the array.
 * @param arr The array to be converted.
 * @return The matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto as_array(MBr<Derived> mat) noexcept {
  return mat.array();
}

template <typename Derived> AX_FORCE_INLINE auto as_array(ABr<Derived> arr) noexcept { return arr; }

/****************************** as_matrix ******************************/

/**
 * @brief Convert an array to a matrix.
 *
 * @tparam Derived The derived type of the array.
 * @param arr The array to be converted.
 * @return The matrix.
 */
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

/**
 * @brief Transpose a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat
 * @return The transposed matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto transpose(DBcr<Derived> mat) noexcept {
  return mat.transpose();
}

/**
 * @brief Transpose a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat
 * @return The transposed matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto transpose(DBr<Derived> mat) noexcept {
  return mat.transpose();
}

/**
 * @brief Transpose a matrix in place.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be transposed.
 */
template <typename Derived> AX_FORCE_INLINE void transpose_(DBr<Derived> mat) noexcept {
  mat.transposeInPlace();
}

/****************************** reshape ******************************/

/**
 * @brief Reshape a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be reshaped.
 * @param rows The number of rows of the reshaped matrix.
 * @param cols The number of columns of the reshaped matrix.
 * @return The reshaped matrix.
 */
template <typename Derived>
AX_FORCE_INLINE auto reshape(MBcr<Derived> mat, idx rows, idx cols) noexcept {
  return mat.derived().reshaped(rows, cols);
}

/**
 * @brief Reshape a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @tparam rows The number of rows of the reshaped matrix.
 * @tparam cols The number of columns of the reshaped matrix.
 * @param mat The matrix to be reshaped.
 * @return The reshaped matrix.
 */
template <typename Derived, idx rows, idx cols>
AX_FORCE_INLINE auto reshape(MBcr<Derived> mat) noexcept {
  return Eigen::Reshaped<const Derived, rows, cols>(mat.derived());
}

/****************************** flatten ******************************/

/**
 * @brief Flatten a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be flattened.
 * @return The flattened matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto flatten(MBcr<Derived> mat) noexcept {
  return Eigen::Reshaped<const Derived, Derived::SizeAtCompileTime, 1>(mat.derived());
}

/**
 * @brief Flatten a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be flattened.
 * @return The flattened matrix.
 */
template <typename Derived> AX_FORCE_INLINE auto flatten(MBr<Derived> mat) noexcept {
  // TODO: test.
  return Eigen::Reshaped<const Derived, Derived::SizeAtCompileTime, 1>(mat.derived());
}

/****************************** field creation ******************************/

/**
 * @brief Create a field.
 *
 * @tparam Derived The derived type of the field.
 * @param dofs The number of degrees of freedom of the field.
 * @return The created field.
 */
template <typename Derived> AX_FORCE_INLINE auto make_field(idx dofs) {
  return field<typename Derived::Scalar, Derived::RowsAtCompileTime>{Derived::RowsAtCompileTime,
                                                                     dofs};
}

// TODO: More create methods.

}  // namespace ax::math
