#include "ax/math/init.hpp"
#include "ax/core/echo.hpp"

#include <Eigen/Core>
#include <thread>
namespace ax::math {

void init_parallel() {
  AX_LOG(INFO) << "Initializing the Eigen Parallel";
  auto cocurrency = std::thread::hardware_concurrency();

  Eigen::initParallel();

  AX_LOG(INFO) << "Number of threads: " << cocurrency;
  omp_set_num_threads(cocurrency);
  Eigen::setNbThreads(cocurrency);

  AX_LOG(INFO) << "Eigen Parallel initialized with " << Eigen::nbThreads() << " threads.";
}

}