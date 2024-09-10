#include "ax/core/entt.hpp"
#include "ax/math/block_matrix/details/cusparse_context.cuh"

namespace ax::math::details {

cusparseContext *get_cusparse_handle() {
  return ensure_resource<CuSparseHandle>().handle_;
}

} // namespace ax::math::details