#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/excepts.hpp"
#include "ax/utils/god.hpp"

#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
#  include <omp.h>
#endif

namespace ax {

namespace details {

template <typename Fn, typename Front, typename... Ts>
void for_each(Fn&& f, BufferView<Front> tsf, BufferView<Ts>... ts) {
  AX_THROW_IF_FALSE(utils::god::all_equal(tsf.Shape(), ts.Shape()...),
                    "All buffers must have the same shape.");
  auto x = tsf.Shape().X();
  auto y = tsf.Shape().Y() == 0 ? 1 : tsf.Shape().Y();
  auto z = tsf.Shape().Z() == 0 ? 1 : tsf.Shape().Z();
  for (size_t k = 0; k < z; ++k) {
    for (size_t j = 0; j < y; ++j) {
      for (size_t i = 0; i < x; ++i) {
        f(tsf(i, j, k), ts(i, j, k)...);
      }
    }
  }
}

template <typename Fn, typename Front, typename... Ts>
void par_for_each(Fn&& f, BufferView<Front> tsf, BufferView<Ts>... ts) {
  AX_THROW_IF_FALSE(utils::god::all_equal(tsf.Shape(), ts.Shape()...),
                    "All buffers must have the same shape.");
  auto x = tsf.Shape().X();
  auto y = tsf.Shape().Y() == 0 ? 1 : tsf.Shape().Y();
  auto z = tsf.Shape().Z() == 0 ? 1 : tsf.Shape().Z();
  size_t total = x * y * z;
#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
  size_t threads = static_cast<size_t>(omp_get_num_threads());
#  pragma omp parallel for schedule(dynamic, (total + threads * 4 - 1) / (threads * 4)) \
      num_threads(threads)
#endif
  for (size_t i = 0; i < total; ++i) {
    size_t k = i / (x * y);
    size_t j = (i - k * x * y) / x;
    size_t l = i - k * x * y - j * x;
    f(tsf(l, j, k), ts(l, j, k)...);
  }
}

template <typename Fn>
AX_HOST_DEVICE AX_CONSTEXPR void for_each_indexed(const Dim<1>& d, Fn&& f) {
  for (size_t i = 0; i < d.X(); ++i) {
    f(i);
  }
}

template <typename Fn>
AX_HOST_DEVICE AX_CONSTEXPR void for_each_indexed(const Dim<2>& d, Fn&& f) {
  for (size_t j = 0; j < d.Y(); ++j) {
    for (size_t i = 0; i < d.X(); ++i) {
      f(i, j);
    }
  }
}

template <typename Fn>
AX_HOST_DEVICE AX_CONSTEXPR void for_each_indexed(const Dim<3>& d, Fn&& f) {
  for (size_t k = 0; k < d.Z(); ++k) {
    for (size_t j = 0; j < d.Y(); ++j) {
      for (size_t i = 0; i < d.X(); ++i) {
        f(i, j, k);
      }
    }
  }
}

template <typename Fn>
void par_for_each_indexed(const Dim<1>& d, Fn&& f) {
  size_t total = d.X();
#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
  size_t threads = static_cast<size_t>(omp_get_num_threads());
#  pragma omp parallel for schedule(dynamic, (total + threads * 4 - 1) / (threads * 4)) \
      num_threads(threads)
#endif
  for (size_t i = 0; i < total; ++i) {
    f(i);
  }
}

template <typename Fn>
void par_for_each_indexed(const Dim<2>& d, Fn&& f) {
  size_t total = d.X() * d.Y();
#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
  size_t threads = static_cast<size_t>(omp_get_num_threads());
#  pragma omp parallel for schedule(dynamic, (total + threads * 4 - 1) / (threads * 4)) \
      num_threads(threads)
#endif
  for (size_t i = 0; i < total; ++i) {
    size_t j = i / d.X();
    size_t l = i - j * d.X();
    f(l, j);
  }
}

template <typename Fn>
void par_for_each_indexed(const Dim<3>& d, Fn&& f) {
  size_t total = d.X() * d.Y() * d.Z();
#if defined AX_HAS_OPENMP && (!defined AX_IS_CUDACC)
  size_t threads = static_cast<size_t>(omp_get_num_threads());
#  pragma omp parallel for schedule(dynamic, (total + threads * 4 - 1) / (threads * 4)) \
      num_threads(threads)
#endif
  for (size_t i = 0; i < total; ++i) {
    size_t k = i / (d.X() * d.Y());
    size_t j = (i - k * d.X() * d.Y()) / d.X();
    size_t l = i - k * d.X() * d.Y() - j * d.X();
    f(l, j, k);
  }
}

}  // namespace details

template <typename Fn, typename Ts>
void for_each(BufferView<Ts> ts, Fn&& f) {
  details::for_each(std::forward<Fn>(f), ts);
}

// Apply f to zipped elements in 123 dimensions.
template <typename Fn, typename... Ts>
void for_each(std::tuple<BufferView<Ts>...> ts, Fn&& f) {
  std::apply(details::for_each<Fn, Ts...>,
             std::tuple_cat(std::forward_as_tuple(std::forward<Fn>(f)), ts));
}

template <typename Fn, typename Ts>
void par_for_each(BufferView<Ts> ts, Fn&& f) {
  details::par_for_each(std::forward<Fn>(f), ts);
}

template <typename Fn, typename... Ts>
void par_for_each(std::tuple<BufferView<Ts>...> ts, Fn&& f) {
  std::apply(details::par_for_each<Fn, Ts...>,
             std::tuple_cat(std::forward_as_tuple(std::forward<Fn>(f)), ts));
}

template <size_t N, typename Fn>
AX_CONSTEXPR AX_FORCE_INLINE void for_each_indexed(const Dim<N>& d, Fn&& f) {
  details::for_each_indexed(d, std::forward<Fn>(f));
}

template <size_t N, typename Fn>
void par_for_each_indexed(const Dim<N>& d, Fn&& f) {
  details::par_for_each_indexed(d, std::forward<Fn>(f));
}

}  // namespace ax