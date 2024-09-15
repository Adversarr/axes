#include "ax/math/utils/init_parallel.hpp"

#include <Eigen/Core>
#include <thread>

#include "ax/core/logging.hpp"

namespace ax::math {

void init_parallel(int num_threads) {
#ifdef AX_HAS_OPENMP
  Eigen::initParallel();
  Eigen::setNbThreads(num_threads > 0 ? num_threads
                                      : static_cast<int>(std::thread::hardware_concurrency()));
  AX_INFO("Eigen Parallel initialized: {} threads", Eigen::nbThreads());
#endif
}

}  // namespace ax::math
