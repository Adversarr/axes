#include "../elasticity_gpu.cuh"
#include "ax/fem/elasticity/stable_neohookean.hpp"

namespace ax::fem {
template class ElasticityCompute_GPU<2, elasticity::StableNeoHookean>;
template class ElasticityCompute_GPU<3, elasticity::StableNeoHookean>;
}  // namespace ax::fem