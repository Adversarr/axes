#pragma once

namespace ax::math {

// Initializes the Eigen Parallel module.
void init_parallel(int num_threads = 0);

}  // namespace ax::math
