#pragma once
#include "common.hpp"

namespace ax::geo {
StatusOr<SurfaceMesh> read_obj(std::string const& path);


}  // namespace ax::geo