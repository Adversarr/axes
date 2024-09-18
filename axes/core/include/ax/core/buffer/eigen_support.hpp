#pragma once
#include "ax/core/excepts.hpp"
#include "ax/math/common.hpp"
#include "buffer_view.hpp"

namespace ax {

namespace details {

template <typename BufT, int outer, int inner>
AX_HOST_DEVICE AX_CONSTEXPR Eigen::Stride<outer, inner> determine_stride(
    const BufferView<BufT>& bufv) {
  AX_DCHECK(bufv.Shape().Z() == 0, "Cannot map a 3D buffer to a 2D Eigen matrix.");
  if constexpr (outer >= 0 && inner >= 0) {
    return {};
  } else if constexpr (outer >= 0 && inner == math::dynamic) {
    const size_t b_x = bufv.Stride().X();
    assert(b_x % sizeof(BufT) == 0);
    const Index inner_rt = b_x / sizeof(BufT);
    return {outer, inner_rt};
  } else if constexpr (outer == math::dynamic && inner >= 0) {
    const size_t b_y = bufv.Stride().Y();
    assert(b_y % sizeof(BufT) == 0);
    const Index outer_rt = static_cast<Index>(b_y == 0 ? 0 : b_y / sizeof(BufT));
    return {outer_rt, inner};
  } else {
    static_assert(outer == math::dynamic && inner == math::dynamic,
                  "Invalid stride template parameters.");
    const size_t b_x = bufv.Stride().X();
    const size_t b_y = bufv.Stride().Y();
    assert(b_x % sizeof(BufT) == 0);
    assert(b_y % sizeof(BufT) == 0);
    const Index inner_rt = b_x / sizeof(BufT);
    const Index outer_rt = static_cast<Index>(b_y == 0 ? 0 : b_y / sizeof(BufT));
    return {outer_rt, inner_rt};
  }
}

template <typename BufT, typename MatrixType, int MapOptions, typename StrideType>
void check_is_valid_scalar_map(const BufferView<BufT>& bufv, StrideType stride) {
  // static check
  using MatrixScalar = typename MatrixType::Scalar;
  static_assert(std::is_same_v<std::remove_const_t<BufT>, MatrixScalar>,
                "Matrix scalar type mismatch.");

  // dynamic checks
  if (bufv.Shape().Z() > 0) {
    // It is not possible to map a 3D buffer to a 2D Eigen matrix, just throw.
    AX_THROW_RUNTIME_ERROR("Cannot map a 3D buffer to a 2D Eigen matrix.");
  }

  // 1. Check the shape match.
  constexpr int rows_at_compile_time = static_cast<int>(MatrixType::RowsAtCompileTime);
  constexpr int cols_at_compile_time = static_cast<int>(MatrixType::ColsAtCompileTime);
  const size_t rows = bufv.Shape().X();  // Our buffer is col-majored.
  const size_t cols = bufv.Shape().Y();
  // Two fairly strange case, you are mapping a buffer to a compile time vector.
  if constexpr (cols_at_compile_time == 1 && rows_at_compile_time != Eigen::Dynamic) {
    const size_t total_size = rows * (cols == 0 ? 1 : cols);  // the buffer length.
    if (total_size != static_cast<size_t>(rows_at_compile_time)) {
      AX_THROW_RUNTIME_ERROR("Buffer size mismatch with the compile time vector size.");
    }
  } else if constexpr (rows_at_compile_time == 1 && cols_at_compile_time != Eigen::Dynamic) {
    const size_t total_size = cols * (rows == 0 ? 1 : rows);  // the buffer length.
    if (total_size != static_cast<size_t>(cols_at_compile_time)) {
      AX_THROW_RUNTIME_ERROR("Buffer size mismatch with the compile time vector size.");
    }
  } else if constexpr (rows_at_compile_time != Eigen::Dynamic) {
    if (rows != static_cast<size_t>(rows_at_compile_time)) {
      AX_THROW_RUNTIME_ERROR(
          "Buffer row size mismatch with the compile time matrix row size. got {} expected {}",
          rows, rows_at_compile_time);
    }
  } else if constexpr (cols_at_compile_time != Eigen::Dynamic) {
    if (cols != static_cast<size_t>(cols_at_compile_time)) {
      AX_THROW_RUNTIME_ERROR(
          "Buffer col size mismatch with the compile time matrix col size. got {} expected {}",
          cols, cols_at_compile_time);
    }
  }

  constexpr int row_stride_compile_time = static_cast<int>(StrideType::InnerStrideAtCompileTime);
  constexpr int col_stride_compile_time = static_cast<int>(StrideType::OuterStrideAtCompileTime);
  // 2. Check the stride match.
  const int row_stride = stride.inner();
  const int col_stride = stride.outer();

  const size_t view_row_stride_bytes = bufv.Stride().X();  // Usually sizeof(T).
  const size_t view_col_stride_bytes = bufv.Stride().Y();

  if constexpr (row_stride_compile_time == 0) {
    // The row is continuous, we need to check the stride of view
    if (view_row_stride_bytes != sizeof(MatrixScalar)) {
      AX_THROW_RUNTIME_ERROR(
          "Buffer row stride mismatch.(expected continuous) got {} expected {}",
          view_row_stride_bytes, sizeof(MatrixScalar));
    }
  } else {
    if (view_row_stride_bytes != row_stride * sizeof(MatrixScalar)) {
      AX_THROW_RUNTIME_ERROR("Buffer row stride mismatch. got {} expected {}",
                               view_row_stride_bytes, row_stride * sizeof(MatrixScalar));
    }
  }

  if constexpr (col_stride_compile_time == 0) {
    // The col is continuous, we need to check the stride of view
    if (view_col_stride_bytes != sizeof(MatrixScalar) * rows) {
      AX_THROW_RUNTIME_ERROR(
          "Buffer col stride mismatch.(expected continuous) got {} expected {}",
          view_col_stride_bytes, sizeof(MatrixScalar) * rows);
    }
  } else {
    if (view_col_stride_bytes != col_stride * sizeof(MatrixScalar)) {
      AX_THROW_RUNTIME_ERROR("Buffer col stride mismatch. got {} expected {}",
                               view_col_stride_bytes, col_stride * sizeof(MatrixScalar));
    }
  }
}

// Lets go back to Eigen::Map. we can map a 2D(3D) buffer to a 1D(2D) view of Vectors (or 1D view of
// matrices) but you have to provide the size of the vector (or matrix) at compile time.
template <typename Scalar, int Rows, int Cols>
class ViewAsEigenMapAdaptor1D {
public:
  using ScalarWithoutConst = std::remove_const_t<Scalar>;
  using MapTWithoutConst = math::Map<math::Matrix<ScalarWithoutConst, Rows, Cols>>;
  using ConstMapT = math::Map<const math::Matrix<ScalarWithoutConst, Rows, Cols>>;
  using MapT = std::conditional_t<std::is_const_v<Scalar>, ConstMapT, MapTWithoutConst>;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor1D()
      = default;  // because BufferView is default constructible.
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor1D(ViewAsEigenMapAdaptor1D const&) = default;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor1D& operator=(ViewAsEigenMapAdaptor1D const&)
      = default;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor1D(ViewAsEigenMapAdaptor1D&&) noexcept = default;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor1D& operator=(ViewAsEigenMapAdaptor1D&&) noexcept
      = default;

  AX_HOST_DEVICE AX_CONSTEXPR MapT operator()(size_t i, size_t j = 0) noexcept {
    return MapT(bufv_.Offset(0, i, j));
  }

  AX_HOST_DEVICE AX_CONSTEXPR ConstMapT operator()(size_t i, size_t j = 0) const noexcept {
    return ConstMapT(bufv_.Offset(0, i, j));
  }

  AX_HOST_DEVICE AX_CONSTEXPR BufferView<Scalar> GetBufferView() const { return bufv_; }

  // TODO: must be private, because we do not check the input here.
  AX_HOST_DEVICE AX_CONSTEXPR explicit ViewAsEigenMapAdaptor1D(BufferView<Scalar> bufv)
      : bufv_(bufv) {}

private:
  BufferView<Scalar> bufv_;
};

template <typename Scalar, int Rows, int Cols>
class ViewAsEigenMapAdaptor2D {
public:
  using MapT = math::Map<math::Matrix<Scalar, Rows, Cols>>;
  using ConstMapT = math::Map<const math::Matrix<Scalar, Rows, Cols>>;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor2D()
      = default;  // because BufferView is default constructible.
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor2D(ViewAsEigenMapAdaptor2D const&) = default;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor2D& operator=(ViewAsEigenMapAdaptor2D const&)
      = default;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor2D(ViewAsEigenMapAdaptor2D&&) noexcept = default;
  AX_HOST_DEVICE AX_CONSTEXPR ViewAsEigenMapAdaptor2D& operator=(ViewAsEigenMapAdaptor2D&&) noexcept
      = default;

  AX_HOST_DEVICE AX_CONSTEXPR MapT operator()(size_t i) noexcept {
    return MapT(bufv_.Offset(0, 0, i));
  }

  AX_HOST_DEVICE AX_CONSTEXPR ConstMapT operator()(size_t i) const noexcept {
    return ConstMapT(bufv_.Offset(0, 0, i));
  }

  AX_HOST_DEVICE AX_CONSTEXPR BufferView<Scalar> GetBufferView() const { return bufv_; }

  // TODO: must be private, because we do not check the input here.
  AX_HOST_DEVICE AX_CONSTEXPR explicit ViewAsEigenMapAdaptor2D(BufferView<Scalar> bufv)
      : bufv_(bufv) {}

private:
  BufferView<Scalar> bufv_;
};

template <typename Scalar, int Rows, int Cols>
void check_valid_eigen_map_1d(const Dim3& shape, const Dim3& stride) {
  static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic, "Dynamic size not supported.");
  constexpr size_t mat_size = Rows * Cols;

  if (is_1d(shape)) {
    AX_THROW_RUNTIME_ERROR("Buffer must be 2D/3D");
  }

  // expect the shape.x is the size of the vector. and stride.x is sizeof(Scalar)
  if (shape.X() != mat_size) {
    AX_THROW_RUNTIME_ERROR("Buffer size mismatch with the compile time vector size.");
  }

  if (stride.X() != sizeof(Scalar)) {
    AX_THROW_RUNTIME_ERROR("Buffer stride mismatch with the compile time vector stride.");
  }
}

// This is not correct if you are mapping to a row-major matrix.
template <typename Scalar, int Rows, int Cols>
void check_valid_eigen_map_2d(const Dim3& shape, const Dim3& stride) {
  static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic, "Dynamic size not supported.");
  if (!is_3d(shape)) {
    AX_THROW_RUNTIME_ERROR("Buffer must be 3D.");
  }

  if (shape.X() != Rows || shape.Y() != Cols) {
    AX_THROW_RUNTIME_ERROR(
        "Buffer size mismatch with the compile time vector size. expect {}x{}, got {}x{}", Rows,
        Cols, shape.X(), shape.Y());
  }

  if (stride.X() != sizeof(Scalar) || stride.Y() != sizeof(Scalar) * Rows) {
    AX_THROW_RUNTIME_ERROR(
        "Buffer stride mismatch with the compile time vector stride. expect {}x{}, got {}x{}",
        sizeof(Scalar), sizeof(Scalar) * Rows, stride.X(), stride.Y());
  }
}

}  // namespace details

/**
 * @brief Given a Eigen Matrix object, create a BufferView.
 */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
AX_HOST_DEVICE AX_CONSTEXPR BufferView<Scalar> view_from_matrix(
    Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& mat) {
  Dim3 shape{static_cast<size_t>(mat.rows()), static_cast<size_t>(mat.cols()), 0};
  return view_from_raw_buffer(mat.data(), shape, BufferDevice::Host);
}

/**
 * @brief Given a Eigen Matrix object, create a BufferView.
 */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
AX_HOST_DEVICE AX_CONSTEXPR BufferView<const Scalar> view_from_matrix(
    Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> const& mat) {
  Dim3 shape{static_cast<size_t>(mat.rows()), static_cast<size_t>(mat.cols()), 0};
  return view_from_raw_buffer(mat.data(), shape, BufferDevice::Host);
}

/**
 * @brief For a 1/2 D BufferView of Vector type, we can view it as a higher dim BufferView of scalar
 */
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
AX_HOST_DEVICE AX_CONSTEXPR BufferView<Scalar> view_as_scalar(
    BufferView<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> bufv,
    char (*)[!std::is_const_v<Scalar>] = nullptr) {
  static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic, "Dynamic size not supported.");
  using MatT = Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>;

  if constexpr (static_cast<bool>(MatT::IsVectorAtCompileTime)) {
    // for 1D, cast to 2D view of scalar.
    // for 2D, cast to 3D view of scalar.
    constexpr size_t size_of_eigen_vec = static_cast<size_t>(MatT::SizeAtCompileTime);
    if (is_1d(bufv.Shape())) {
      Dim3 stride{sizeof(Scalar), bufv.Stride().X(), 0};
      Dim3 shape{size_of_eigen_vec, bufv.Shape().X(), 0};
      return BufferView<Scalar>(reinterpret_cast<Scalar*>(bufv.Data()->data()), shape, stride,
                                bufv.Device());
    } else if (is_2d(bufv.Shape())) {
      Dim3 stride{sizeof(Scalar), bufv.Stride().X(), bufv.Stride().Y()};
      Dim3 shape{size_of_eigen_vec, bufv.Shape().X(), bufv.Shape().Y()};
      return BufferView<Scalar>(reinterpret_cast<Scalar*>(bufv.Data()->data()), shape, stride,
                                bufv.Device());
    }
    AX_THROW_RUNTIME_ERROR("For vector type Buffer must be 1D or 2D to view as scalar.");
    AX_UNREACHABLE();
  } else {
    if (!is_1d(bufv.Shape())) {
      AX_THROW_RUNTIME_ERROR("Buffer of matrix type must be 1D to view as scalar.");
    }
    // Matrix inside, we can view it as a 3D buffer.
    constexpr size_t size_at_compile_time = (Options & Eigen::RowMajor) ? Cols : Rows;
    Dim3 stride{sizeof(Scalar),                         // stride.x is always sizeof(Scalar)
                sizeof(Scalar) * size_at_compile_time,  // stride.y is the outer size, we need bytes
                bufv.Stride().X()};                     // stride.z is the stride of view
    Dim3 shape{((Options & Eigen::RowMajor) ? Cols : Rows),
               ((Options & Eigen::RowMajor) ? Rows : Cols), bufv.Shape().X()};
    return BufferView<Scalar>(reinterpret_cast<Scalar*>(bufv.Data()->data()), shape, stride,
                              bufv.Device());
  }
}

/**
 * @brief Given a BufferView, create a corresponding Eigen::Map.
 *
 */
template <typename MatrixType, typename StrideType = Eigen::Stride<0, 0>,
          int MapOptions = Eigen::Unaligned, typename BufT>
AX_HOST_DEVICE AX_CONSTEXPR auto view_as_matrix_full(BufferView<BufT> bufv,
                                                     char (*)[std::is_scalar_v<BufT>] = nullptr) {
  using MapT = Eigen::Map<
      std::conditional_t<std::is_const_v<BufT>, std::add_const_t<MatrixType>, MatrixType>,
      MapOptions, StrideType>;
  StrideType stride = details::determine_stride<BufT, StrideType::OuterStrideAtCompileTime,
                                                StrideType::InnerStrideAtCompileTime>(bufv);
  details::check_is_valid_scalar_map<BufT, MatrixType, MapOptions, StrideType>(bufv, stride);
  return MapT(bufv.Data(), static_cast<Index>(bufv.Shape().X()),
              static_cast<Index>(bufv.Shape().Y()), stride);
}

/**
 * @brief Given a BufferView, create a corresponding Eigen::Map.
 *
 */
template <typename MatrixType, typename StrideType = Eigen::Stride<0, 0>,
          int MapOptions = Eigen::Unaligned, typename BufT>
AX_HOST_DEVICE AX_CONSTEXPR auto view_as_matrix_full(BufferView<BufT> bufv,
                                                     char (*)[!std::is_scalar_v<BufT>] = nullptr) {
  // BufT is a Eigen::Matrix, but compile time
  assert(is_1d(bufv.Shape()));
  constexpr int buft_rows = BufT::RowsAtCompileTime;
  constexpr int buft_cols = BufT::ColsAtCompileTime;
  constexpr int mapped_rows_at_compile_time_expected = buft_rows * buft_cols;
  using ScalarType = typename BufT::Scalar;
  static_assert(std::is_same_v<ScalarType, typename MatrixType::Scalar>,
                "Matrix scalar type mismatch.");
  static_assert(BufT::RowsAtCompileTime > 0 && BufT::ColsAtCompileTime > 0,
                "Compile time matrix must have a fixed size.");
  static_assert(MatrixType::RowsAtCompileTime == Eigen::Dynamic
                    || MatrixType::RowsAtCompileTime == mapped_rows_at_compile_time_expected,
                "Matrix row size mismatch with the compile time matrix row size.");
  static_assert(StrideType::InnerStrideAtCompileTime == 0,
                "Inner stride must be continuous for compile time matrix.");
  const size_t outer_stride_in_bytes_runtime = bufv.Stride().X();
  constexpr int outer_stride_provided = StrideType::OuterStrideAtCompileTime;

  using MapT = Eigen::Map<
      std::conditional_t<std::is_const_v<BufT>, std::add_const_t<MatrixType>, MatrixType>,
      MapOptions, StrideType>;

  if constexpr (outer_stride_provided == 0) {
    // expect continuous.
    if (outer_stride_in_bytes_runtime
        != mapped_rows_at_compile_time_expected * sizeof(ScalarType)) {
      AX_THROW_RUNTIME_ERROR("Buffer outer stride mismatch. got {} expected {}",
                               outer_stride_in_bytes_runtime,
                               mapped_rows_at_compile_time_expected * sizeof(ScalarType));
    }
  } else if constexpr (outer_stride_provided != Eigen::Dynamic) {
    if (outer_stride_in_bytes_runtime != outer_stride_provided * sizeof(ScalarType)) {
      AX_THROW_RUNTIME_ERROR("Buffer outer stride mismatch. got {} expected {}",
                               outer_stride_in_bytes_runtime,
                               outer_stride_provided * sizeof(ScalarType));
    }
  }
  if constexpr (outer_stride_provided != Eigen::Dynamic) {
    return MapT(bufv.Data()->data(), mapped_rows_at_compile_time_expected, bufv.Shape().X());
  } else {
    return MapT(bufv.Data()->data(), mapped_rows_at_compile_time_expected, bufv.Shape().X(),
                StrideType{0, outer_stride_in_bytes_runtime / sizeof(ScalarType)});
  }
}

/**
 * @brief Take the first dimension (x-dim) as the size of the vector/matrix and the rest as the
 *        the view's input.
 */
template <typename Scalar, int Rows, int Cols = 1>
AX_HOST_DEVICE AX_CONSTEXPR details::ViewAsEigenMapAdaptor1D<Scalar, Rows, Cols> view_as_matrix_1d(
    BufferView<Scalar> bufv) {
  details::check_valid_eigen_map_1d<Scalar, Rows, Cols>(bufv.Shape(), bufv.Stride());
  return details::ViewAsEigenMapAdaptor1D<Scalar, Rows, Cols>(bufv);
}

/**
 * @brief Take the first two dimension (xy-dim) as the size of the vector/matrix and the third as
 * the the view's input.
 */
template <typename Scalar, int Rows, int Cols>
AX_HOST_DEVICE AX_CONSTEXPR details::ViewAsEigenMapAdaptor2D<Scalar, Rows, Cols> view_as_matrix_2d(
    BufferView<Scalar> bufv) {
  details::check_valid_eigen_map_2d<Scalar, Rows, Cols>(bufv.Shape(), bufv.Stride());
  return details::ViewAsEigenMapAdaptor2D<Scalar, Rows, Cols>(bufv);
}

}  // namespace ax