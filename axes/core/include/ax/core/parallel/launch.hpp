#pragma once

#include <functional>
#include <taskflow/algorithm/for_each.hpp> // IWYU pragma: export
#include <taskflow/algorithm/reduce.hpp>   // IWYU pragma: export
#include <taskflow/taskflow.hpp>

#include "ax/core/buffer/buffer_view.hpp"
#include "block.hpp"

namespace ax::parallel {
namespace details {

template <size_t Nb>
AX_CONSTEXPR size_t linear_index(const Dim<Nb>& dim, const Dim<Nb>& idx) {
  if constexpr (Nb == 1) {
    return idx.sizes_[0];
  } else if constexpr (Nb == 2) {
    return idx.sizes_[0] + idx.sizes_[1] * dim.sizes_[0];
  } else if constexpr (Nb == 3) {
    return idx.sizes_[0] + idx.sizes_[1] * dim.sizes_[0]
           + idx.sizes_[2] * dim.sizes_[0] * dim.sizes_[1];
  } else {
    static_assert(Nb == 1 || Nb == 2 || Nb == 3, "Unsupported dimension.");
    AX_UNREACHABLE();
  }
}

template <size_t Nb>
AX_CONSTEXPR Dim<Nb> from_linear_index(const Dim<Nb>& dim, size_t linear_id) {
  if constexpr (Nb == 1) {
    return Dim<Nb>(linear_id);
  } else if constexpr (Nb == 2) {
    return Dim<Nb>(linear_id % dim.sizes_[0], linear_id / dim.sizes_[0]);
  } else if constexpr (Nb == 3) {
    return Dim<Nb>(linear_id % dim.sizes_[0], (linear_id / dim.sizes_[0]) % dim.sizes_[1],
                   linear_id / (dim.sizes_[0] * dim.sizes_[1]));
  } else {
    static_assert(Nb == 1 || Nb == 2 || Nb == 3, "Unsupported dimension.");
    AX_UNREACHABLE();
  }
}

template <size_t Nb>
AX_CONSTEXPR size_t product(const Dim<Nb>& dim) {
  size_t result = dim.sizes_[0];
  for (size_t i = 1; i < Nb; ++i) {
    result *= dim.sizes_[i];
  }
  return result;
}

template <typename Fn, size_t Ng, size_t Nb>
AX_FORCE_INLINE auto invoke_fn(Fn&& fn, const Dim<Ng>& grid_id, const Dim<Nb>& block_id,
                               const ExecParam<Ng, Nb>& param) {
  if constexpr (std::is_invocable_v<Fn, const Dim<Ng>&, const Dim<Nb>&>) {
    return fn(grid_id, block_id);  /// direct call.
  } else if constexpr (std::is_invocable_v<Fn, size_t, size_t>) {
    const size_t block_linear_id = linear_index(param.block_size_, block_id);
    const size_t grid_linear_id = linear_index(param.grid_size_, grid_id);
    return fn(grid_linear_id, block_linear_id);
  } else if constexpr (std::is_invocable_v<Fn, size_t>) {
    const size_t block_linear_id = linear_index(param.block_size_, block_id);
    const size_t grid_linear_id = linear_index(param.grid_size_, grid_id);
    const size_t block_size = product(param.block_size_);
    return fn(block_linear_id + grid_linear_id * block_size);
  } else {
    static_assert(!std::is_same_v<Fn, Fn>, "Unsupported function signature.");
  }
}

template <typename Fn, typename... Ts, size_t... Idx>
AX_FORCE_INLINE auto invoke_fn_buffer(Fn&& f, std::tuple<BufferView<Ts>...>& tup,
                                      std::index_sequence<Idx...>, size_t arg) {
  return f(std::get<Idx>(tup)(arg)...);
}

template <typename Fn, typename... Ts, size_t... Idx>
AX_FORCE_INLINE auto invoke_fn_buffer(Fn&& f, std::tuple<BufferView<Ts>...>& tup,
                                      std::index_sequence<Idx...>, size_t arg1, size_t arg2) {
  return f(std::get<Idx>(tup)(arg1, arg2)...);
}

template <typename Fn, typename... Ts, size_t... Idx>
AX_FORCE_INLINE auto invoke_fn_buffer(Fn&& f, std::tuple<BufferView<Ts>...>& tup,
                                      std::index_sequence<Idx...>, size_t arg1, size_t arg2,
                                      size_t arg3) {
  return f(std::get<Idx>(tup)(arg1, arg2, arg3)...);
}

template <typename Flow, typename Fn, size_t Ng, size_t Nb>
AX_FORCE_INLINE tf::Task for_each_index_(Flow& flow, const ExecParam<Ng, Nb>& param, Fn fn) {
  size_t grid_size_total = product(param.grid_size_);
  return flow.template for_each_index<size_t, size_t, size_t>(
      0, grid_size_total, 1, [fn, param](size_t grid_linear_id) {
        auto grid_id = details::from_linear_index(param.grid_size_, grid_linear_id);
        // Run in zyx order.
        if constexpr (Nb == 1) {
          for (size_t x = 0; x < param.block_size_.sizes_[0]; ++x) {
            details::invoke_fn(fn, grid_id, Dim<1>(x), param);
          }
        } else if constexpr (Nb == 2) {
          for (size_t y = 0; y < param.block_size_.sizes_[1]; ++y) {
            for (size_t x = 0; x < param.block_size_.sizes_[0]; ++x) {
              details::invoke_fn(fn, grid_id, Dim<2>(x, y), param);
            }
          }
        } else if constexpr (Nb == 3) {
          for (size_t z = 0; z < param.block_size_.sizes_[2]; ++z) {
            for (size_t y = 0; y < param.block_size_.sizes_[1]; ++y) {
              for (size_t x = 0; x < param.block_size_.sizes_[0]; ++x) {
                details::invoke_fn(fn, grid_id, Dim<3>(x, y, z), param);
              }
            }
          }
        } else {
          static_assert(Nb == 1 || Nb == 2 || Nb == 3, "Unsupported dimension.");
        }
      });
}

}  // namespace details

class TfBuilder {
public:
  // For CPU paralleling, we take each grid as a thread. each block in thread are executed in
  // serial. example: grid_size=[2, 2, 2], and block_size=[32, 32, 32], we will launch 2x2x2 tasks.
  /**
   * @brief Add a index-based for-each task.
   *
   * @tparam Fn
   * @tparam Ng
   * @tparam Nb
   * @param param
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, size_t Ng, size_t Nb>
  tf::Task ForEachIndex(const ExecParam<Ng, Nb>& param, Fn fn) {
    return details::for_each_index_(flow_, param, fn);
  }

  /**
   * @brief Add a task to execute fn for each element in buf.
   *
   * @tparam Fn
   * @tparam T
   * @param buf
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename T>
  tf::Task ForEach1D(BufferView<T> buf, Fn fn) {
    auto grid_size = Dim<1>(buf.Shape().X());
    auto block_size = Dim<1>(1);
    return ForEachIndex(ExecParam<1, 1>(grid_size, block_size),
                        [fn, buf](const Dim<1>& grid, const Dim<1>& /* block */) {
                          fn(buf(grid.sizes_[0]));
                        });
  }

  /**
   * @brief Add a task to execute fn for each element in zipped buffers.
   *
   * @tparam Fn
   * @tparam Ts
   * @param buf
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename... Ts>
  tf::Task ForEach1D(std::tuple<BufferView<Ts>...> buf, Fn fn) {
    // TODO: check shape.
    auto grid_size = Dim<1>(std::get<0>(buf).Shape().X());
    auto block_size = Dim<1>(1);
    return ForEachIndex(ExecParam<1, 1>(grid_size, block_size),
                        [fn, buf](const Dim<1>& grid, const Dim<1>& /* block */) {
                          details::invoke_fn_buffer(fn, buf, std::index_sequence_for<Ts...>(),
                                                    grid.sizes_[0]);
                        });
  }

  /**
   * @brief Add a task to execute fn for each element in 2D buffer.
   *
   * @tparam Fn
   * @tparam Front
   * @param f
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename Front>
  tf::Task ForEach2D(BufferView<Front> f, Fn fn) {
    if (!ax::details::is_2d(f.Shape())) {
      throw std::invalid_argument("Unsupported dimension.");
    }
    auto grid_size = Dim<2>(f.Shape().X(), f.Shape().Y());
    auto block_size = Dim<1>(1);
    return ForEachIndex(ExecParam<2, 1>(grid_size, block_size),
                        [fn, f](const Dim<2>& grid, const Dim<1>& /* block */) {
                          fn(f(grid.sizes_[0], grid.sizes_[1]));
                        });
  }

  /**
   * @brief Add a task to execute fn for each element in zipped 2D buffers.
   *
   * @tparam Fn
   * @tparam Ts
   * @param buf
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename... Ts>
  tf::Task ForEach2D(std::tuple<BufferView<Ts>...> buf, Fn fn) {
    auto grid_size = Dim<2>(std::get<0>(buf).Shape().X(), std::get<0>(buf).Shape().Y());
    auto block_size = Dim<1>(1);
    return ForEachIndex(ExecParam<2, 1>(grid_size, block_size),
                        [fn, buf](const Dim<2>& grid, const Dim<1>& /* block */) {
                          auto buf_copy = buf;  // Fxxk mutable.
                          details::invoke_fn_buffer(fn, buf_copy, std::index_sequence_for<Ts...>(),
                                                    grid.sizes_[0], grid.sizes_[1]);
                        });
  }

  /**
   * @brief Add a task to execute fn for each element in 3D buffer.
   *
   * @tparam Fn
   * @tparam Front
   * @param f
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename Front>
  tf::Task ForEach3D(BufferView<Front> f, Fn fn) {
    if (!ax::details::is_3d(f.Shape())) {
      throw std::invalid_argument("Unsupported dimension.");
    }
    auto grid_size = Dim<3>(f.Shape().X(), f.Shape().Y(), f.Shape().Z());
    auto block_size = Dim<1>(1);
    return ForEachIndex(ExecParam<3, 1>(grid_size, block_size),
                        [fn, f](const Dim<3>& grid, const Dim<1>& /* block */) {
                          auto f_copy = f;  // Fxxk mutable.
                          fn(f_copy(grid.sizes_[0], grid.sizes_[1], grid.sizes_[2]));
                        });
  }

  /**
   * @brief Add a task to execute fn for each element in zipped 3D buffer.
   *
   * @tparam Fn
   * @param f
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename... Ts>
  tf::Task ForEach3D(std::tuple<BufferView<Ts>...> buf, Fn fn) {
    auto grid_size = Dim<3>(std::get<0>(buf).Shape().X(), std::get<0>(buf).Shape().Y(),
                            std::get<0>(buf).Shape().Z());
    auto block_size = Dim<1>(1);
    return ForEachIndex(ExecParam<3, 1>(grid_size, block_size),
                        [fn, buf](const Dim<3>& grid, const Dim<1>& /* block */) {
                          auto buf_copy = buf;  // Fxxk mutable.
                          details::invoke_fn_buffer(fn, buf_copy, std::index_sequence_for<Ts...>(),
                                                    grid.sizes_[0], grid.sizes_[1], grid.sizes_[2]);
                        });
  }

  /**
   * @brief Generic ForEach function, will dispatch to ForEach1D, ForEach2D, or ForEach3D.
   *
   * @tparam Fn
   * @tparam T
   * @param b
   * @param fn
   * @return tf::Task
   */
  template <typename Fn, typename T>
  tf::Task ForEach(BufferView<T> b, Fn fn) {
    if (ax::details::is_1d(b.Shape())) {
      return ForEach1D(b, fn);
    } else if (ax::details::is_2d(b.Shape())) {
      return ForEach2D(b, fn);
    } else if (ax::details::is_3d(b.Shape())) {
      return ForEach3D(b, fn);
    } else {
      throw make_invalid_argument("Unsupported dimension.");
    }
  }

  // Another important parallel pattern is to transform then reduce. the most common case is to
  // calculate the dot product.
  // i.e. reduce(transform(Idx), ...)
  // TODO: Partitioner?
  template <typename ValueType, typename TransformFn, typename ReduceFn, size_t Ng, size_t Nb>
  tf::Task TransformReduceIndex(const ExecParam<Ng, Nb>& param, ValueType& inout,
                                TransformFn transform_fn, ReduceFn reduce_fn, ValueType zero = {}) {
    return flow_.emplace([&inout, param, transform_fn, reduce_fn, zero](tf::Subflow& sbf) {
      std::vector<ValueType> results(prod(param.grid_size_), zero);
      auto transform_task = details::for_each_index_(
          sbf, param,
          [param, transform_fn, reduce_fn, &results](const Dim<Ng>& grid_id,
                                                     const Dim<Nb>& block_id) {
            size_t grid_linear_id = details::linear_index(param.grid_size_, grid_id);
            auto& result = results[grid_linear_id];
            result = reduce_fn(result, details::invoke_fn(transform_fn, grid_id, block_id, param));
          });
      auto reduce_task = sbf.reduce(results.begin(), results.end(), inout, reduce_fn);
      transform_task.precede(reduce_task);
      sbf.join();
    });
  }

  template <typename ValueType, typename ReduceFn, size_t Ng, size_t Nb>
  tf::Task ReduceIndex(const ExecParam<Ng, Nb>& param, ValueType& inout, ReduceFn reduce_fn,
                       ValueType zero = {}) {
    return TransformReduceIndex(param, inout, std::identity{},  // transform_fn: do nothing
                                reduce_fn, zero);
  }

  template <typename ValueType, typename TranformFn, typename ReduceFn, typename... Tp>
  tf::Task TransformReduce1D(std::tuple<BufferView<Tp>...> buf, ValueType& inout,
                             TranformFn transform_fn, ReduceFn reduce_fn, ValueType zero = {}) {
    auto grid_size = Dim<1>(std::get<0>(buf).Shape().X());
    auto block_size = Dim<1>(1);
    return TransformReduceIndex(
        ExecParam<1, 1>(grid_size, block_size), inout,
        [transform_fn, buf](const Dim<1>& grid, const Dim<1>& /* block */) {
          auto buf_copy = buf;  // Fxxk mutable.
          return details::invoke_fn_buffer(transform_fn, buf_copy, std::index_sequence_for<Tp...>{},
                                           grid.sizes_[0]);
        },
        reduce_fn, zero);
  }

  template <typename ValueType, typename TransformFn, typename ReduceFn, typename... Tp>
  tf::Task TransformReduce2D(std::tuple<BufferView<Tp>...> buf, ValueType& inout,
                             TransformFn transform_fn, ReduceFn reduce_fn, ValueType zero = {}) {
    // TODO: check shape.

    auto grid_size = Dim<2>(std::get<0>(buf).Shape().X(), std::get<0>(buf).Shape().Y());
    auto block_size = Dim<1>(1);
    return TransformReduceIndex(
        ExecParam<2, 1>(grid_size, block_size), inout,
        [transform_fn = transform_fn, buf](const Dim<2>& grid, const Dim<1>& /* block */) {
          auto buf_copy = buf;  // Fxxk mutable.
          return details::invoke_fn_buffer(transform_fn, buf_copy, std::index_sequence_for<Tp...>{},
                                           grid.sizes_[0], grid.sizes_[1]);
        },
        reduce_fn, zero);
  }

  template <typename ValueType, typename TransformFn, typename ReduceFn, typename... Tp>
  tf::Task TransformReduce3D(std::tuple<BufferView<Tp>...> buf, ValueType& inout,
                             TransformFn transform_fn, ReduceFn reduce_fn, ValueType zero = {}) {
    // TODO: check shape.

    auto grid_size = Dim<3>(std::get<0>(buf).Shape().X(), std::get<0>(buf).Shape().Y(),
                            std::get<0>(buf).Shape().Z());
    auto block_size = Dim<1>(1);
    return TransformReduceIndex(
        ExecParam<3, 1>(grid_size, block_size), inout,
        [transform_fn, buf](const Dim<3>& grid, const Dim<1>& /* block */) {
          auto buf_copy = buf;  // Fxxk mutable.
          return details::invoke_fn_buffer(transform_fn, buf_copy, std::index_sequence_for<Tp...>{},
                                           grid.sizes_[0], grid.sizes_[1], grid.sizes_[2]);
        },
        reduce_fn, zero);
  }

  template <typename ValueType, typename TransformFn, typename ReduceFn, typename... Tp>
  tf::Task TransformReduce(std::tuple<BufferView<Tp>...> buf, ValueType& inout,
                           TransformFn transform_fn, ReduceFn reduce_fn, ValueType zero = {}) {
    if (ax::details::is_1d(std::get<0>(buf).Shape())) {
      return TransformReduce1D(buf, inout, transform_fn, reduce_fn, zero);
    } else if (ax::details::is_2d(std::get<0>(buf).Shape())) {
      return TransformReduce2D(buf, inout, transform_fn, reduce_fn, zero);
    } else if (ax::details::is_3d(std::get<0>(buf).Shape())) {
      return TransformReduce3D(buf, inout, transform_fn, reduce_fn, zero);
    } else {
      throw make_invalid_argument("Unsupported dimension.");
    }
  }

  tf::Taskflow Build() noexcept {
    tf::Taskflow flow;
    std::swap(flow, flow_);
    return flow;
  }

  template <typename... Args>
  auto Emplace(Args&&... args) {
    return flow_.emplace(std::forward<Args>(args)...);
  }

private:
  tf::Taskflow flow_;
};

inline tf::Executor make_executor(size_t num_threads) {
  return tf::Executor{num_threads};
}

inline tf::Executor make_executor() {
  return tf::Executor{};
}

}  // namespace ax::parallel