#pragma once
#include <memory>
#include <vector>

#include "ax/core/config.hpp"
#include "ax/core/macros.hpp"
#include "common.hpp"

namespace ax::math {

template<typename T, typename Allocator=std::allocator<T>>
class FieldData {
public:
  AX_FORCE_INLINE explicit FieldData(idx size) noexcept;

  AX_FORCE_INLINE void Resize(idx new_size) noexcept;

  AX_FORCE_INLINE idx Size() const noexcept;

protected:
  std::vector<T, Allocator> data_;
};

template <typename T, typename Allocator, idx dim>
class FieldAccessor {
};

template <typename T, typename Allocator, idx dim>
FieldAccessor<T, Allocator, dim> make_accessor(FieldData<T, Allocator> & field, veci<dim> const& shape) {
  return FieldAccessor<T, Allocator, dim>(field, shape);
}

}
