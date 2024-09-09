/**
 * @file common.hpp
 * @brief Common math utilities and types for axes library.
 */
#pragma once

#include <Eigen/Core>
#include <type_traits>
#include <vector>

#include "ax/core/common.hpp"
#include "ax/utils/dup_tuple.hpp"

namespace ax::math {

/****************************** Vectors ******************************/

/**
 * @brief Vector type.
 *
 * @tparam Scalar The scalar type of the vector.
 * @tparam dim The dimension of the vector.
 */
template <typename Scalar, int dim>
using Vector = Eigen::Vector<Scalar, dim>;
template <typename Scalar, int dim>
using RowVector = Eigen::RowVector<Scalar, dim>;

/****************************** Helper Class and Constants ******************************/

/**
 * @brief Constant representing dynamic size.
 */
constexpr int dynamic = Eigen::Dynamic;

/**
 * @brief Helper type aliases for Eigen types.
 */
template <typename T>
using DB = Eigen::DenseBase<T>;
template <typename T>
using DBr = DB<T> &;
template <typename T>
using DBcr = DB<T> const &;
template <typename T>
using MB = Eigen::MatrixBase<T>;
template <typename T>
using MBr = MB<T> &;
template <typename T>
using MBcr = MB<T> const &;
template <typename T>
using AB = Eigen::ArrayBase<T>;
template <typename T>
using ABr = AB<T> &;
template <typename T>
using ABcr = AB<T> const &;

/**
 * @brief Most commonly used is `standard` RealVector.
 */
template <int dim>
using RealVector = Vector<Real, dim>;
template <int dim>
using FloatVector = Vector<float, dim>;
using RealVector2
    = RealVector<2>;  ///< Alias for 2D vector with double precision floating point number
using RealVector3
    = RealVector<3>;  ///< Alias for 3D vector with double precision floating point number
using RealVector4
    = RealVector<4>;  ///< Alias for 4D vector with double precision floating point number
using RealVectorX
    = RealVector<dynamic>;  ///< Alias for vector with double precision float point number
using FloatVector2
    = FloatVector<2>;  ///< Alias for 2D vector with single precision floating point number
using FloatVector3
    = FloatVector<3>;  ///< Alias for 3D vector with single precision floating point number
using FloatVector4
    = FloatVector<4>;  ///< Alias for 4D vector with single precision floating point number
using FloatVectorX
    = FloatVector<dynamic>;  ///< Alias for vector with single precision float point number

/**
 * @brief Index vector type.
 *
 * @tparam dim The dimension of the Index vector.
 */
template <int dim>
using IndexVector = Vector<Index, dim>;

using IndexVectorX = IndexVector<dynamic>;
using IndexVector2 = IndexVector<2>;  ///< Alias for 2D Index vector.
using IndexVector3 = IndexVector<3>;  ///< Alias for 3D Index vector.
using IndexVector4 = IndexVector<4>;  ///< Alias for 4D Index vector.

/****************************** Matrices ******************************/

/**
 * @brief Default matrix type: Auto aligned, and if no template param is provided, the default is
 * dynamic size.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 */
template <typename Scalar, int rows, int cols>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

template <int rows, int cols>
using RealMatrix = Matrix<Real, rows, cols>;
template <int rows, int cols>
using FloatMatrix = Matrix<float, rows, cols>;
template <int rows, int cols>
using IndexMatrix = Matrix<Index, rows, cols>;
using RealMatrixX = RealMatrix<dynamic, dynamic>;
using FloatMatrixX = FloatMatrix<dynamic, dynamic>;
using IndexMatrixX = IndexMatrix<dynamic, dynamic>;

using RealMatrix2 = RealMatrix<2, 2>;
using RealMatrix3 = RealMatrix<3, 3>;
using RealMatrix4 = RealMatrix<4, 4>;
using FloatMatrix2 = FloatMatrix<2, 2>;
using FloatMatrix3 = FloatMatrix<3, 3>;
using FloatMatrix4 = FloatMatrix<4, 4>;
using IndexMatrix2 = IndexMatrix<2, 2>;
using IndexMatrix3 = IndexMatrix<3, 3>;
using IndexMatrix4 = IndexMatrix<4, 4>;

/****************************** Field ******************************/

/**
 * @brief Field is the most important type in axes.
 */
template <typename Scalar, int dim>
using Field = Matrix<Scalar, dim, dynamic>;

template <int dim>
using RealField = Field<Real, dim>;
template <int dim>
using IndexField = Field<Index, dim>;

using RealField1 = RealField<1>;
using RealField2 = RealField<2>;
using RealField3 = RealField<3>;
using RealField4 = RealField<4>;
using RealFieldX = RealField<dynamic>;

using IndexField1 = IndexField<1>;
using IndexField2 = IndexField<2>;
using IndexField3 = IndexField<3>;
using IndexField4 = IndexField<4>;
using IndexFieldX = IndexField<dynamic>;

/******************************************************************************
 * Matrix Meta data getter.
 *****************************************************************************/

/**
 * @brief Alias for the shape of a matrix.
 */
using MatShape = std::pair<Index, Index>;

/**
 * @brief Get the number of rows of a matrix at compile time.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of rows of the matrix.
 */
template <typename Derived>
AX_CONSTEVAL AX_HOST_DEVICE Index rows_static(const Eigen::EigenBase<Derived> &) {
  return Derived::RowsAtCompileTime;
}

/**
 * @brief Get the number of columns of a matrix at compile time.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of columns of the matrix.
 */
template <typename Derived>
AX_CONSTEVAL AX_HOST_DEVICE Index cols_static(const Eigen::EigenBase<Derived> &) {
  return Derived::ColsAtCompileTime;
}

/**
 * @brief Get the number of rows of a matrix at runtime.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of rows of the matrix.
 */
template <typename Derived>
AX_CONSTEXPR AX_HOST_DEVICE Index rows(const Eigen::EigenBase<Derived> &mat) {
  return mat.rows();
}

/**
 * @brief Get the number of columns of a matrix at runtime.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix.
 * @return The number of columns of the matrix.
 */
template <typename Derived>
AX_CONSTEXPR AX_HOST_DEVICE Index cols(const Eigen::EigenBase<Derived> &mat) {
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
AX_HOST_DEVICE AX_FORCE_INLINE MatShape shape_of(const Eigen::EigenBase<Derived> &mat) {
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
AX_HOST_DEVICE AX_FORCE_INLINE MatShape shape_of_static(const Eigen::EigenBase<Derived> &mat) {
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
template <typename T, size_t... seq>
AX_HOST_DEVICE AX_FORCE_INLINE Vector<T, sizeof...(seq)> tuple_to_vector_impl(
    const utils::details::dup_tuple<T, sizeof...(seq)> &tuple, std::index_sequence<seq...>) {
  return Vector<T, sizeof...(seq)>{std::get<seq>(tuple)...};
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
AX_HOST_DEVICE AX_FORCE_INLINE utils::DupTuple<T, sizeof...(seq)> vector_to_tuple_impl(
    const Vector<T, sizeof...(seq)> &vec, std::index_sequence<seq...>) {
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
template <typename T, int dim>
AX_HOST_DEVICE AX_FORCE_INLINE Vector<T, dim> tuple_to_vector(
    const utils::DupTuple<T, dim> &tuple) {
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
AX_HOST_DEVICE AX_FORCE_INLINE utils::DupTuple<T, dim> vector_to_tuple(const Vector<T, dim> &vec) {
  return details::vector_to_tuple_impl<T>(vec, std::make_index_sequence<dim>());
}

/****************************** Constants ******************************/

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/**
 * @brief The constant pi.
 */
template <typename Scalar = Real>
constexpr Scalar pi = static_cast<Scalar>(M_PI);

/**
 * @brief The constant pi in radians.
 */
template <typename Scalar = Real>
constexpr Scalar pi_radian = static_cast<Scalar>(0.017453292519943);

/**
 * @brief The constant NaN (Not-a-Number).
 */
template <typename Scalar = Real>
constexpr Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();

/**
 * @brief The constant infinity.
 */
template <typename Scalar = Real>
constexpr Scalar inf = std::numeric_limits<Scalar>::infinity();

/**
 * @brief The constant epsilon.
 */
template <typename Scalar = Real>
constexpr Scalar epsilon = std::numeric_limits<Scalar>::epsilon();

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
template <int rows, int cols = 1, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto ones() {
  return Matrix<Scalar, rows, cols>::Ones();
}

/**
 * @brief Create a matrix filled with ones.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with ones.
 */
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto ones(Index cols) {
  return Matrix<Scalar, rows, dynamic>::Ones(rows, cols);
}

/**
 * @brief Create a matrix filled with ones.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with ones.
 */
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto ones(Index rows, Index cols) {
  return Matrix<Scalar, dynamic, dynamic>::Ones(rows, cols);
}

/**
 * @brief Fill a matrix with ones.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam r The number of rows of the matrix.
 * @tparam c The number of columns of the matrix.
 * @param mat The matrix to be filled.
 */
template <typename Scalar, int r, int c>
AX_HOST_DEVICE AX_FORCE_INLINE void ones_(const Matrix<Scalar, r, c> &mat) {
  mat.setOnes();
}

/**
 * @brief Fill a scalar with ones.
 *
 * @tparam Scalar The scalar type.
 * @param mat The scalar to be filled.
 */
template <typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
AX_HOST_DEVICE AX_FORCE_INLINE void ones_(const Scalar &mat) {
  mat.setOnes();
}

/**
 * @brief Create a ones value.
 *
 * @tparam T The type of the value.
 * @return A ones value.
 */
template <typename T>
AX_HOST_DEVICE AX_FORCE_INLINE T make_ones() {
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
template <int rows, int cols = 1, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto zeros() {
  return Matrix<Scalar, rows, cols>::Zero();
}

/**
 * @brief Create a matrix filled with zeros.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with zeros.
 */
template <Index rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto zeros(Index cols) {
  return Matrix<Scalar, rows, dynamic>::Zero(rows, cols);
}

/**
 * @brief Create a matrix filled with zeros.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return A matrix filled with zeros.
 */
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto zeros(Index rows, Index cols) {
  return Matrix<Scalar, dynamic, dynamic>::Zero(rows, cols);
}

/**
 * @brief Fill a matrix with zeros.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam r The number of rows of the matrix.
 * @tparam c The number of columns of the matrix.
 * @param mat The matrix to be filled.
 */
template <typename Scalar, int r, int c>
AX_HOST_DEVICE AX_FORCE_INLINE void zeros_(const Matrix<Scalar, r, c> &mat) {
  mat.setZero();
}

/**
 * @brief Fill a scalar with zeros.
 *
 * @tparam Scalar The scalar type.
 * @param mat The scalar to be filled.
 */
template <typename Scalar, typename = std::enable_if_t<std::is_arithmetic_v<Scalar>>>
AX_HOST_DEVICE AX_FORCE_INLINE void zeros_(const Scalar &mat) {
  mat.setZero();
}

/**
 * @brief Create a zeros value.
 *
 * @tparam T The type of the value.
 * @return A zeros value.
 */
template <typename T>
AX_HOST_DEVICE AX_FORCE_INLINE T make_zeros() {
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
template <int rows, int cols = 1, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto constant(Scalar value) {
  return Matrix<Scalar, rows, cols>::Constant(value);
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
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto constant(Scalar value, Index cols) {
  return Matrix<Scalar, rows, dynamic>::Constant(rows, cols, value);
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
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto constant(Scalar value, Index rows, Index cols) {
  return Matrix<Scalar, dynamic, dynamic>::Constant(rows, cols, value);
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
template <typename Scalar, int r, int c>
AX_HOST_DEVICE AX_FORCE_INLINE void constant_(const Matrix<Scalar, r, c> &mat, Scalar value) {
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
AX_HOST_DEVICE AX_FORCE_INLINE void constant_(const Scalar &mat, Scalar value) {
  mat.setConstant(value);
}

/**
 * @brief Create a constant value.
 *
 * @tparam T The type of the value.
 * @param value The constant value.
 * @return A constant value.
 */
template <typename T, typename Scalar>
AX_HOST_DEVICE AX_FORCE_INLINE T make_constant(Scalar value) {
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
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto linspace(Scalar start, Scalar end) {
  return Vector<Scalar, rows>::LinSpaced(rows, start, end);
}

/**
 * @brief Create a vector filled with linearly spaced values.
 *
 * @tparam Scalar The scalar type of the vector.
 * @param end The end value.
 * @return A vector filled with linearly spaced values.
 */
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto linspace(Scalar end) {
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
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto linspace(Scalar start, Scalar end, Index rows) {
  return Vector<Scalar, dynamic>::LinSpaced(rows, start, end);
}

/****************************** 5. arange ******************************/

/**
 * @brief Create a vector filled with values from start to stop.
 *
 * @tparam Scalar The scalar type of the vector.
 * @param stop The stop value.
 * @return A vector filled with values from start to stop.
 */
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto arange(Index stop) {
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
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto arange(Index start, Index stop) {
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
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto eye() {
  return Matrix<Scalar, rows, rows>::Identity();
}

/**
 * @brief Create an identity matrix.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An identity matrix.
 */
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto eye(Index rows) {
  return Matrix<Scalar, dynamic, dynamic>::Identity(rows, rows);
}

/**
 * @brief Create an identity matrix.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An identity matrix.
 */
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto identity() {
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
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto identity(Index rows) {
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
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto diag(
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
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto diag(
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
AX_HOST_DEVICE AX_FORCE_INLINE auto diag(
    MBcr<Derived> mat,
    char (*)[Derived::RowsAtCompileTime == Derived::ColsAtCompileTime] = nullptr) {
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
template <int rows, int cols = 1, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto empty() {
  return Matrix<Scalar, rows, cols>{};
}

/**
 * @brief Create an empty matrix.
 *
 * @tparam rows The number of rows of the matrix.
 * @tparam Scalar The scalar type of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An empty matrix.
 */
template <int rows, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto empty(Index cols) {
  return Matrix<Scalar, rows, dynamic>(rows, cols);
}

/**
 * @brief Create an empty matrix.
 *
 * @tparam Scalar The scalar type of the matrix.
 * @param rows The number of rows of the matrix.
 * @param cols The number of columns of the matrix.
 * @return An empty matrix.
 */
template <typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto empty(Index rows, Index cols) {
  return Matrix<Scalar, dynamic, dynamic>{rows, cols};
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
template <int dim, typename Scalar = Real>
AX_HOST_DEVICE AX_FORCE_INLINE auto unit(Index i) {
  return Vector<Scalar, dim>::Unit(i);
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
template <typename Derived>
AX_FORCE_INLINE auto eval(DBcr<Derived> mat) {
  return mat.eval();
}

/****************************** cast ******************************/

/**
 * @brief Cast a matrix to a different type.
 *
 * @tparam To The type to cast to.
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be cast.
 * @return The casted matrix.
 */
template <typename To, typename Derived>
AX_FORCE_INLINE auto cast(DBcr<Derived> mat) {
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
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_array(MBcr<Derived> mat) noexcept {
  return mat.array();
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_array(ABcr<Derived> arr) noexcept {
  return arr;
}

/**
 * @brief Convert an array to a matrix.
 *
 * @tparam Derived The derived type of the array.
 * @param arr The array to be converted.
 * @return The matrix.
 */
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_array(MBr<Derived> mat) noexcept {
  return mat.array();
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_array(ABr<Derived> arr) noexcept {
  return arr;
}

/****************************** as_matrix ******************************/

/**
 * @brief Convert an array to a matrix.
 *
 * @tparam Derived The derived type of the array.
 * @param arr The array to be converted.
 * @return The matrix.
 */
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_matrix(ABcr<Derived> arr) noexcept {
  return arr.matrix();
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_matrix(MBcr<Derived> mat) noexcept {
  return mat;
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_matrix(ABr<Derived> arr) noexcept {
  return arr.matrix();
}

template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto as_matrix(MBr<Derived> mat) noexcept {
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
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto transpose(DBcr<Derived> mat) noexcept {
  return mat.transpose();
}

/**
 * @brief Transpose a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat
 * @return The transposed matrix.
 */
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto transpose(DBr<Derived> mat) noexcept {
  return mat.transpose();
}

/**
 * @brief Transpose a matrix in place.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be transposed.
 */
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE void transpose_(DBr<Derived> mat) noexcept {
  mat.transposeInPlace();
}

/****************************** reshape ******************************/

/**
 * @brief Reshape a matrix: This could cause runtime cost.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be reshaped.
 * @param rows The number of rows of the reshaped matrix.
 * @param cols The number of columns of the reshaped matrix.
 * @return The reshaped matrix.
 */
template <typename Derived>
AX_HOST AX_FORCE_INLINE auto reshape(MBcr<Derived> mat, Index rows, Index cols) noexcept {
  // NOLINTBEGIN: The first parameter is used but reported unused.
  return mat.template reshaped<Eigen::AutoOrder>(rows, cols);
  // NOLINTEND
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
template <int rows, int cols, typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE Eigen::Reshaped<const Derived, rows, cols> reshape(
    MBcr<Derived> mat) noexcept {
  return {mat.derived()};
}

/**
 * @brief Reshape a matrix: will only create a reshaped view.
 *
 * @tparam Derived The derived type of the matrix.
 * @tparam rows The number of rows of the reshaped matrix.
 * @param mat The matrix to be reshaped.
 * @param cols The number of columns of the reshaped matrix.
 * @return The reshaped matrix.
 */
template <int rows, typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE Eigen::Reshaped<const Derived, rows, dynamic> reshape(
    MBcr<Derived> mat, Index cols) noexcept {
  return Eigen::Reshaped<const Derived, rows, dynamic>(mat.derived(), rows, cols);
}

/****************************** flatten ******************************/

/**
 * @brief Flatten a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be flattened.
 * @return The flattened matrix.
 */
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto flatten(MBcr<Derived> mat) noexcept {
  return Eigen::Reshaped<const Derived, Derived::SizeAtCompileTime, 1>(mat.derived());
}

/**
 * @brief Flatten a matrix.
 *
 * @tparam Derived The derived type of the matrix.
 * @param mat The matrix to be flattened.
 * @return The flattened matrix.
 */
template <typename Derived>
AX_HOST_DEVICE AX_FORCE_INLINE auto flatten(MBr<Derived> mat) noexcept {
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
template <typename Derived>
AX_FORCE_INLINE auto make_field(Index dofs) {
  return Field<typename Derived::Scalar, Derived::RowsAtCompileTime>{Derived::RowsAtCompileTime,
                                                                     dofs};
}

template <int rows>
AX_FORCE_INLINE RealField<rows> make_real_field(Index dofs) {
  return RealField<rows>{rows, dofs};
}

template <int rows>
AX_FORCE_INLINE IndexField<rows> make_index_field(Index dofs) {
  return IndexField<rows>{rows, dofs};
}

/**
 * @brief Aligned allocated vector.
 */
template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

inline std::string to_string(Eigen::ComputationInfo info) {
  switch (info) {
    case Eigen::ComputationInfo::NumericalIssue:
      return "NumericalIssue";
    case Eigen::ComputationInfo::NoConvergence:
      return "NoConvergence";
    case Eigen::ComputationInfo::InvalidInput:
      return "InvalidInput";
    case Eigen::ComputationInfo::Success:
      return "Success";
    default:
      return "Unknown";
  }
}

template <typename Mapped, int Options = Eigen::Unaligned,
          typename StrideType = Eigen::Stride<0, 0>>
using Map = Eigen::Map<Mapped, Options, StrideType>;

template <typename Mapped>
using DefaultMap = Map<Mapped, Eigen::Unaligned, Eigen::Stride<0, 0>>;

}  // namespace ax::math