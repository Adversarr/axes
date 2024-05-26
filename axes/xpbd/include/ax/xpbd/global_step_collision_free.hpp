#pragma once
#include "ax/xpbd/common.hpp" // IWYU pragma: export

namespace ax::xpbd {

void global_step_collision_free(math::field3r const& weighted_position, math::field1r const& weights);

}