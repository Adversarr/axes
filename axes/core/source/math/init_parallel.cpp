#include "ax/math/init_parallel.hpp"

#include <Eigen/Core>
#include <thread>

#include "ax/core/logging.hpp"
namespace ax::math {

void init_parallel(int nT) {
#ifdef AX_HAS_OPENMP
  Eigen::initParallel();
  Eigen::setNbThreads(nT > 0 ? nT : static_cast<int>(std::thread::hardware_concurrency()));
  AX_INFO("Eigen Parallel initialized: {} threads", Eigen::nbThreads());
#endif
}

}  // namespace ax::math
