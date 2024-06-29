#include "ax/math/init.hpp"
#include "ax/core/echo.hpp"

#include <Eigen/Core>
#include <thread>
namespace ax::math {

void init_parallel(int nT) {
#ifdef AX_HAS_OPENMP
  AX_LOG(INFO) << "Initializing the Eigen Parallel";
  Eigen::initParallel();
  Eigen::setNbThreads(nT > 0 ? nT : static_cast<int>(std::thread::hardware_concurrency()));
  AX_LOG(INFO) << "Eigen Parallel initialized with " << Eigen::nbThreads() << " threads.";
#endif
}

}
