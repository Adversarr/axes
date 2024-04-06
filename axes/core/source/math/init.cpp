#include "ax/math/init.hpp"
#include "ax/core/echo.hpp"

#include <Eigen/Core>
#include <thread>
namespace ax::math {

void init_parallel() {
#ifdef AX_HAS_OPENMP
  AX_LOG(INFO) << "Initializing the Eigen Parallel";
  Eigen::initParallel();
  AX_LOG(INFO) << "Eigen Parallel initialized with " << Eigen::nbThreads() << " threads.";
#endif
}

}
