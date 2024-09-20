#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/utils/god.hpp"

#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
#  include <omp.h>
#endif

namespace ax {
namespace details {

template <typename TransformFn, typename ReduceFn, typename ValueType, typename Front,
          typename... Ts>
AX_CONSTEXPR AX_FORCE_INLINE ValueType transform_reduce_dispatch(TransformFn&& f, ReduceFn&& g,
                                                                 ValueType init,
                                                                 BufferView<Front> tsf,
                                                                 BufferView<Ts>... ts) {
  auto x = tsf.Shape().X();
  auto y = tsf.Shape().Y() == 0 ? 1 : tsf.Shape().Y();
  auto z = tsf.Shape().Z() == 0 ? 1 : tsf.Shape().Z();
  ValueType result = init;
  for (size_t k = 0; k < z; ++k) {
    for (size_t j = 0; j < y; ++j) {
#pragma unroll
      for (size_t i = 0; i < x; ++i) {
        result = g(result, f(tsf(i, j, k), ts(i, j, k)...));
      }
    }
  }
  return result;
}

template <typename TransformFn, typename ReduceFn, typename ValueType, typename Front,
          typename... Ts>
ValueType par_transform_reduce_dispatch(TransformFn&& f, ReduceFn&& g, ValueType init,
                                        BufferView<Front> front, BufferView<Ts>... ts) {
#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
  AX_THROW_IF_FALSE(utils::god::all_equal(ts.Shape()...), "All buffers must have the same shape.");
  auto x = front.Shape().X();
  auto y = front.Shape().Y() == 0 ? 1 : front.Shape().Y();
  auto z = front.Shape().Z() == 0 ? 1 : front.Shape().Z();
  ValueType result = init;
  size_t total = x * y * z;
  std::vector<ValueType> results(omp_get_num_threads(), init);
#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
  size_t threads = static_cast<size_t>(omp_get_num_threads());
#pragma omp parallel for schedule(dynamic, (total + threads * 4 - 1) / (threads * 4)) num_threads(threads)
#endif
  for (size_t i = 0; i < total; ++i) {
    size_t k = i / (x * y);
    size_t j = (i - k * x * y) / x;
    size_t l = i - k * x * y - j * x;
    results[omp_get_thread_num()]
        = g(results[omp_get_thread_num()], f(front(l, j, k), ts(l, j, k)...));
  }
  for (auto& r : results) {
    result = g(result, r);
  }
  return result;
#else
  return transform_reduce_dispatch(std::forward<TransformFn>(f), std::forward<ReduceFn>(g), init,
                                   front, ts...);
#endif
}

}  // namespace details

template <typename ValueType, typename TranformFn, typename ReduceFn, typename Front>
AX_CONSTEXPR AX_FORCE_INLINE auto transform_reduce(BufferView<Front> ts, ValueType init,
                                                   TranformFn&& f, ReduceFn&& g) {
  return details::transform_reduce_dispatch(std::forward<TranformFn>(f), std::forward<ReduceFn>(g),
                                            init, ts);
}

template <typename ValueType, typename TranformFn, typename ReduceFn, typename... Ts>
AX_CONSTEXPR AX_FORCE_INLINE auto transform_reduce(std::tuple<BufferView<Ts>...> ts, ValueType init,
                                                   TranformFn&& f, ReduceFn&& g) {
  return std::apply(
      details::transform_reduce_dispatch<TranformFn, ReduceFn, ValueType, Ts...>,
      std::tuple_cat(
          std::forward_as_tuple(std::forward<TranformFn>(f), std::forward<ReduceFn>(g), init), ts));
}

template <typename ValueType, typename TranformFn, typename ReduceFn, typename Front>
auto par_transform_reduce(BufferView<Front> ts, ValueType init, TranformFn&& f, ReduceFn&& g) {
  return details::par_transform_reduce_dispatch(std::forward<TranformFn>(f),
                                                std::forward<ReduceFn>(g), init, ts);
}

template <typename ValueType, typename TranformFn, typename ReduceFn, typename... Ts>
auto par_transform_reduce(std::tuple<BufferView<Ts>...> ts, ValueType init, TranformFn&& f,
                          ReduceFn&& g) {
  return std::apply(
      details::par_transform_reduce_dispatch<TranformFn, ReduceFn, ValueType, Ts...>,
      std::tuple_cat(
          std::forward_as_tuple(std::forward<TranformFn>(f), std::forward<ReduceFn>(g), init), ts));
}

}  // namespace ax