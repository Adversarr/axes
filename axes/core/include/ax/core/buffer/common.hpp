#pragma once
#include "ax/core/common.hpp"
#include "ax/core/dim.hpp"
#include "ax/utils/enum_refl.hpp"

namespace ax {

AX_DEFINE_ENUM_CLASS(BufferDevice, Host, Device);

template <typename T>
class Buffer;

// Holds a view over a buffer, does not own the memory.
template <typename T>
class BufferView;

// Buffer dimension type, use dim3 instead of DimVariant for better performance
using BufferDim = Dim3;

}  // namespace ax
