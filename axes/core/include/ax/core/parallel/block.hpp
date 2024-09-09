#pragma once
#include "ax/core/dim.hpp"

namespace ax::parallel {

template <size_t Ng, size_t Nb>
struct ExecParam {
  Dim<Ng> grid_size_;
  Dim<Nb> block_size_;

  AX_CONSTEXPR ExecParam(Dim<Ng> grid_size, Dim<Nb> block_size)
      : grid_size_(grid_size), block_size_(block_size) {}
};

}  // namespace ax::parallel