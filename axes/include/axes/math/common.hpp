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
template <int dim = Eigen::Dynamic, typename Scalar = Real> using Vec = Eigen::Vector<Scalar, dim>;

template <int dim = Eigen::Dynamic, typename Scalar = Real> using RowVec
    = Eigen::RowVector<Scalar, dim>;

/**
 * @brief Most commonly used is `standard` RealVector.
 *
 */
using Vec2r = Vec<2>;               ///< Alias for 2D vector with double precision
                                    ///< floating point number.
using Vec3r = Vec<3>;               ///< Alias for 3D vector with double precision
                                    ///< floating point number.
using Vec4r = Vec<4>;               ///< Alias for 4D vector with double precision
                                    ///< floating point number.
using VecXr = Vec<Eigen::Dynamic>;  ///< Alias for vector with double precision
                                    ///< floating point number.

/**
 * @brief index vector type.
 *
 * @tparam dim The dimension of the index vector.
 */
template <Index dim> using Veci = Vec<dim, Index>;
using VecXi = Vec<Eigen::Dynamic, Index>;
using Vec2i = Vec<2, Index>;  ///< Alias for 2D index vector.
using Vec3i = Vec<3, Index>;  ///< Alias for 3D index vector.
using Vec4i = Vec<4, Index>;  ///< Alias for 4D index vector.

/****************************** Matrices ******************************/

/**
 * @brief Default matrix type: Auto aligned, and if no template param is
 * provided, the default is dynamic size
 *
 * @tparam Scalar The scalar type of the matrix.
 * @tparam rows The number of rows of the matrix.
 * @tparam cols The number of columns of the matrix.
 */
template <Index rows = Eigen::Dynamic, Index cols = Eigen::Dynamic, typename Scalar = Real>
using Mat = Eigen::Matrix<Scalar, rows, cols>;

///< Alias for matrix with double precision floating point number.
using MatXXr = Mat<Eigen::Dynamic, Eigen::Dynamic, Real>;

template <typename Scalar = Real> using Mat2x2 = Mat<2, 2, Scalar>;  ///< Alias for 2x2 matrix.
template <typename Scalar = Real> using Mat3x3 = Mat<3, 3, Scalar>;  ///< Alias for 3x3 matrix.
template <typename Scalar = Real> using Mat4x4 = Mat<4, 4, Scalar>;  ///< Alias for 4x4 matrix.

using Mat2Xr = Mat<2, Eigen::Dynamic, Real>;  ///< Alias for 2xN matrix with double precision
                                              ///< floating point number.
using Mat3Xr = Mat<3, Eigen::Dynamic, Real>;  ///< Alias for 3xN matrix with double precision
                                              ///< floating point number.
using Mat4Xr = Mat<4, Eigen::Dynamic, Real>;  ///< Alias for 4xN matrix with double precision
                                              ///< floating point number.

/**
 * @brief Alias to commonly used matrices.
 *
 */
using Mat2x2R = Mat<2, 2>;  ///< Alias for 1x2 matrix with double precision
                            ///< floating point number.
using Mat2x3R = Mat<2, 3>;  ///< Alias for 2x3 matrix with double precision
                            ///< floating point number.
using Mat2x4R = Mat<2, 4>;  ///< Alias for 2x4 matrix with double precision
                            ///< floating point number.
using Mat3x2R = Mat<3, 2>;  ///< Alias for 3x2 matrix with double precision
                            ///< floating point number.
using Mat3x3R = Mat<3, 3>;  ///< Alias for 3x3 matrix with double precision
                            ///< floating point number.
using Mat3x4R = Mat<3, 4>;  ///< Alias for 3x4 matrix with double precision
                            ///< floating point number.
using Mat4x2R = Mat<4, 2>;  ///< Alias for 4x2 matrix with double precision
                            ///< floating point number.
using Mat4x3R = Mat<4, 3>;  ///< Alias for 4x3 matrix with double precision
                            ///< floating point number.
using Mat4x4R = Mat<4, 4>;  ///< Alias for 4x4 matrix with double precision
                            ///< floating point number.

/****************************** Field ******************************/

/**
 * @brief Field is the most important type in axes.
 */
template <Index dims, typename Scalar = Real> using Field = Mat<dims, Eigen::Dynamic, Scalar>;

template <typename Scalar = Real> using Field1 = Field<1, Scalar>;
template <typename Scalar = Real> using Field2 = Field<2, Scalar>;
template <typename Scalar = Real> using Field3 = Field<3, Scalar>;
template <typename Scalar = Real> using Field4 = Field<4, Scalar>;

using Field1r = Field1<Real>;
using Field2r = Field2<Real>;
using Field3r = Field3<Real>;
using Field4r = Field4<Real>;

using Field1i = Field1<Index>;
using Field2i = Field2<Index>;
using Field3i = Field3<Index>;
using Field4i = Field4<Index>;

/****************************** Scalar Type For ******************************/

namespace details {

template <typename T, bool = std::is_scalar_v<T>> struct ScalarTypeFor;

template <typename T> struct ScalarTypeFor<T, false> {
  using type = typename T::Scalar;
};

template <typename T> struct ScalarTypeFor<T, true> {
  using type = T;
};

template <typename T> using ScalarTypeForT = typename ScalarTypeFor<std::decay_t<T>>::type;

}  // namespace details
/**
 * Meta programming utilities for matrices
 */
namespace math {

template <typename T> struct Trait {
  static constexpr int rows = T::RowsAtCompileTime;
  static constexpr int cols = T::ColsAtCompileTime;
  static constexpr bool is_vector = T::IsVectorAtCompileTime;
  static constexpr bool is_row_major = T::IsRowMajor;
  static constexpr bool is_col_major = !is_row_major;
  static constexpr bool is_scalar = false;
  using Scalar = typename T::Scalar;
  static constexpr bool is_dense = true;
};

template <> struct Trait<Float32> {
  static constexpr int rows = 1;
  static constexpr int cols = 1;
  static constexpr int dim = rows * cols;
  static constexpr bool is_vector = false;
  static constexpr bool is_row_major = false;
  static constexpr bool is_col_major = !is_row_major;
  static constexpr bool is_scalar = true;
  static constexpr bool is_dense = true;
  using Scalar = Float32;
};
template <> struct Trait<Float64> {
  static constexpr int rows = 1;
  static constexpr int cols = 1;
  static constexpr bool is_vector = false;
  static constexpr int dim = rows * cols;
  static constexpr bool is_row_major = false;
  static constexpr bool is_col_major = !is_row_major;
  static constexpr bool is_scalar = true;
  static constexpr bool is_dense = true;
  using Scalar = Float64;
};

/******************************************************************************
 * Matrix Meta data getter.
 *****************************************************************************/

using MatShape = std::pair<Index, Index>;

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEVAL Index rows_static(const Eigen::EigenBase<Derived> &) {
  return Derived::RowsAtCompileTime;
}

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEVAL Index cols_static(const Eigen::EigenBase<Derived> &) {
  return Derived::ColsAtCompileTime;
}

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEXPR Index rows(const Eigen::EigenBase<Derived> &mat) {
  return mat.rows();
}

template <typename Derived>
AXES_FORCE_INLINE AXES_CONSTEXPR Index cols(const Eigen::EigenBase<Derived> &mat) {
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
template <typename T, size_t... seq> AXES_FORCE_INLINE Vec<sizeof...(seq), T> tuple_to_vector_impl(
    const utils::details::dup_tuple<T, sizeof...(seq)> &tuple, std::index_sequence<seq...>) {
  return Vec<sizeof...(seq), T>{std::get<seq>(tuple)...};
}
}  // namespace details

template <size_t dim> using IndexTuple
    = utils::DupTuple<Index, dim>;  ///< Alias for tuple of duplicated indices.

/**
 * Convert a DupTuple to a vector.
 * @tparam T The type of the elements in the DupTuple.
 * @tparam dim The dimension of the DupTuple.
 * @param tuple The DupTuple to be converted.
 * @return A vector with the same elements as the DupTuple.
 */
template <typename T, size_t dim>
AXES_FORCE_INLINE Vec<dim, T> tuple_to_vector(const utils::DupTuple<T, dim> &tuple) {
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
template <typename Scalar = Real> constexpr Scalar pi = static_cast<Scalar>(M_PI);

/****************************** Common Ops ******************************/

/****************************** 1. ones ******************************/

template <Index rows, Index cols = 1, typename Scalar = Real> AXES_FORCE_INLINE auto ones() {
  return Mat<rows, cols, Scalar>::Ones();
}

template <Index rows, typename Scalar = Real> AXES_FORCE_INLINE auto ones(Index cols) {
  return Mat<rows, Eigen::Dynamic, Scalar>::Ones(rows, cols);
}

template <typename Scalar = Real> AXES_FORCE_INLINE auto ones(Index rows, Index cols) {
  return Mat<Eigen::Dynamic, Eigen::Dynamic, Scalar>::Ones(rows, cols);
}

/****************************** 2. zeros ******************************/

template <Index rows, Index cols = 1, typename Scalar = Real> AXES_FORCE_INLINE auto zeros() {
  return Mat<rows, cols, Scalar>::Zero();
}

template <Index rows, typename Scalar = Real> AXES_FORCE_INLINE auto zeros(Index cols) {
  return Mat<rows, Eigen::Dynamic, Scalar>::Zero(rows, cols);
}

template <typename Scalar = Real> AXES_FORCE_INLINE auto zeros(Index rows, Index cols) {
  return Mat<Eigen::Dynamic, Eigen::Dynamic, Scalar>::Zero(rows, cols);
}

/****************************** 3. constant ******************************/

template <Index rows, Index cols = 1, typename Scalar = Real>
AXES_FORCE_INLINE auto constant(Scalar value) {
  return Mat<rows, cols, Scalar>::Constant(value);
}

template <Index rows, typename Scalar = Real>
AXES_FORCE_INLINE auto constant(Scalar value, Index cols) {
  return Mat<rows, Eigen::Dynamic, Scalar>::Constant(rows, cols, value);
}

template <typename Scalar = Real>
AXES_FORCE_INLINE auto constant(Scalar value, Index rows, Index cols) {
  return Mat<Eigen::Dynamic, Eigen::Dynamic, Scalar>::Constant(rows, cols, value);
}

/****************************** 4. linspace ******************************/

template <Index rows, typename Scalar = Real>
AXES_FORCE_INLINE auto linspace(Scalar start, Scalar end) {
  return Vec<rows, Scalar>::LinSpaced(Eigen::Sequential, rows, start, end);
}

template <Index rows, typename Scalar = Real> AXES_FORCE_INLINE auto linspace(Scalar end) {
  return Vec<rows, Scalar>::LinSpaced(Eigen::Sequential, rows, 0, end);
}

template <typename Scalar = Real>
AXES_FORCE_INLINE auto linspace(Scalar start, Scalar end, Index rows) {
  return Vec<Eigen::Dynamic, Scalar>::LinSpaced(Eigen::Sequential, rows, start, end);
}

/****************************** 5. arange ******************************/

template <typename Scalar = Real> AXES_FORCE_INLINE auto arange(Index stop) {
  return linspace<Scalar>(0, stop);
}

template <typename Scalar = Real> AXES_FORCE_INLINE auto arange(Index start, Index stop) {
  return linspace<Scalar>(start, stop);
}

/****************************** 6. eye ******************************/

template <Index rows, typename Scalar = Real> AXES_FORCE_INLINE auto eye() {
  return Mat<rows, rows, Scalar>::Identity();
}

template <typename Scalar = Real> AXES_FORCE_INLINE auto eye(Index rows) {
  return Mat<Eigen::Dynamic, Eigen::Dynamic, Scalar>::Identity(rows, rows);
}

template <Index rows, typename Scalar = Real> AXES_FORCE_INLINE auto identity() {
  return Mat<rows, rows, Scalar>::Identity();
}

template <typename Scalar = Real> AXES_FORCE_INLINE auto identity(Index rows) {
  return Mat<Eigen::Dynamic, Eigen::Dynamic, Scalar>::Identity(rows, rows);
}

/****************************** 7. diag ******************************/

template <typename Derived>
AXES_FORCE_INLINE auto diag(const Eigen::DenseBase<Derived> &mat,
                            char (*)[Derived::ColsAtCompileTime == 1] = nullptr) {
  return mat.asDiagonal();
}

template <typename Derived>
AXES_FORCE_INLINE auto diag(const Eigen::DenseBase<Derived> &mat,
                            char (*)[Derived::RowsAtCompileTime == 1] = nullptr) {
  return mat.transpose().asDiagonal();
}

template <typename Derived>
AXES_FORCE_INLINE auto diag(const Eigen::DenseBase<Derived> &mat,
                            char (*)[Derived::RowsAtCompileTime == Derived::ColsAtCompileTime]
                            = nullptr) {
  return mat.diagonal();
}

/****************************** 8. empty ******************************/

template <Index rows, Index cols = 1, typename Scalar = Real> AXES_FORCE_INLINE auto empty() {
  return Mat<rows, cols, Scalar>{};
}

template <Index rows, typename Scalar = Real> AXES_FORCE_INLINE auto empty(Index cols) {
  return Mat<rows, Eigen::Dynamic, Scalar>{rows, cols};
}

template <typename Scalar = Real> AXES_FORCE_INLINE auto empty(Index rows, Index cols) {
  return Mat<Eigen::Dynamic, Eigen::Dynamic, Scalar>{rows, cols};
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
template<typename Derived>
AXES_FORCE_INLINE auto eval(const Eigen::DenseBase<Derived> &mat) {
  return mat.eval();
}

}  // namespace axes
