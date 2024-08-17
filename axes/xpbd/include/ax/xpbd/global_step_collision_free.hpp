#pragma once
#include "ax/xpbd/common.hpp" // IWYU pragma: export

namespace ax::xpbd {

void global_step_collision_free(math::RealField3 const& weighted_position, math::RealField1 const& weights);

}