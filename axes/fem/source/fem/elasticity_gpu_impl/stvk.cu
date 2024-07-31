#include "../elasticity_gpu.cuh"
#include "ax/fem/elasticity/stvk.hpp"

namespace ax::fem {
template class ElasticityCompute_GPU<2, elasticity::StVK>;
template class ElasticityCompute_GPU<3, elasticity::StVK>;
}  // namespace ax::fem