#pragma once
#include <tuple>

#include "common.hpp"
#include "structure_binding.hpp"

namespace ax::math {
namespace details {

struct UnpackAdaptorBase {};

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct LRefUnpackAdaptor : UnpackAdaptorBase {
  using type = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
  static_assert(type::IsVectorAtCompileTime, "Only vectors are supported");

  explicit LRefUnpackAdaptor(type& data) : data_(data) {}

  template <size_t I> _Scalar& get() noexcept { return data_[I]; }
  template <size_t I> const _Scalar& get() const noexcept { return data_[I]; }

  type& data_;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct ConstLRefUnpackAdaptor : UnpackAdaptorBase {
  using type = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
  static_assert(type::IsVectorAtCompileTime, "Only vectors are supported");

  explicit ConstLRefUnpackAdaptor(const type& data) : data_(data) {}

  template <size_t I> _Scalar& get() noexcept { return data_[I]; }
  template <size_t I> const _Scalar& get() const noexcept { return data_[I]; }

  const type& data_;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct RRefUnpackAdaptor : UnpackAdaptorBase {
  using type = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
  static_assert(type::IsVectorAtCompileTime, "Only vectors are supported");

  explicit RRefUnpackAdaptor(type&& data) : data_(std::move(data)) {}

  template <size_t I> _Scalar& get() noexcept { return data_[I]; }
  template <size_t I> const _Scalar& get() const noexcept { return data_[I]; }

  type data_;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct ConstRRefUnpackAdaptor : UnpackAdaptorBase {
  using type = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
  static_assert(type::IsVectorAtCompileTime, "Only vectors are supported");

  explicit ConstRRefUnpackAdaptor(const type&& data) : data_(std::move(data)) {}

  template <size_t I> _Scalar& get() noexcept { return data_[I]; }
  template <size_t I> const _Scalar& get() const noexcept { return data_[I]; }

  type data_;
};
}  // namespace details

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
auto unpack(Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& data) {
  return details::LRefUnpackAdaptor<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>{data};
}

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
auto unpack(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& data) {
  return details::ConstLRefUnpackAdaptor<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>{data};
}

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
auto unpack(Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>&& data) {
  return details::RRefUnpackAdaptor<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>{
      std::move(data)};
}

template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
auto unpack(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>&& data) {
  return details::ConstRRefUnpackAdaptor<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>{
      std::move(data)};
}

}  // namespace ax::math

namespace std {
#define AX_DECLARE_UNPACK_ADAPTOR(Adaptor)                                                    \
  template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> \
  struct tuple_size<                                                                          \
      ax::math::details::Adaptor<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>> {      \
    using type = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;          \
    static constexpr size_t value = type::SizeAtCompileTime;                                  \
  };                                                                                          \
  template <size_t I, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,     \
            int _MaxCols>                                                                     \
  struct tuple_element<                                                                       \
      I, ax::math::details::Adaptor<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>> {   \
    using type = _Scalar;                                                                     \
  };

AX_DECLARE_UNPACK_ADAPTOR(LRefUnpackAdaptor)
AX_DECLARE_UNPACK_ADAPTOR(ConstLRefUnpackAdaptor)
AX_DECLARE_UNPACK_ADAPTOR(RRefUnpackAdaptor)
AX_DECLARE_UNPACK_ADAPTOR(ConstRRefUnpackAdaptor)

#undef AX_DECLARE_UNPACK_ADAPTOR

}  // namespace std
