#pragma once

#include <Eigen/Core>

#include "acore/common.hpp"

namespace axes::math {

/**
 * @brief Vector type.
 */
template <typename Scalar = Real, int dim = Eigen::Dynamic> using Vector
    = Eigen::Vector<Scalar, dim>;
template <typename Scalar = Real> using Vector2 = Vector<Scalar, 2>;
template <typename Scalar = Real> using Vector3 = Vector<Scalar, 3>;
template <typename Scalar = Real> using Vector4 = Vector<Scalar, 4>;

/**
 * @brief index vector type.
 */
template <int dim = Eigen::Dynamic> using IndexVector = Vector<Index, dim>;
using IndexVector2 = Vector<Index, 2>;
using IndexVector3 = Vector<Index, 2>;
using IndexVector4 = Vector<Index, 2>;

/**
 * @brief Default matrix type: Auto aligned, and if no template param is
 * provided, the default is dynamic size
 */
template <typename Scalar = Real, int rows = Eigen::Dynamic,
          int cols = Eigen::Dynamic>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;
template <typename Scalar = Real> using Mat2x2 = Matrix<Scalar, 2, 2>;
template <typename Scalar = Real> using Mat3x3 = Matrix<Scalar, 3, 3>;
template <typename Scalar = Real> using Mat4x4 = Matrix<Scalar, 4, 4>;


/**
 * Meta programming utilities for matrices
 */

/**
 * @brief Returns compile-time column
 *
 * @tparam T 
 * @return 
 */
template <typename T> constexpr Eigen::Index get_cols() {
  using Type = std::remove_cvref_t<T>;
  return Type::ColsAtCompileTime;
}

/**
 * @brief Returns compile-time rows
 *
 * @tparam T 
 * @return 
 */
template <typename T> constexpr Eigen::Index get_rows() {
  using Type = std::remove_cvref_t<T>;
  return Type::RowsAtCompileTime;
}

}  // namespace axes
