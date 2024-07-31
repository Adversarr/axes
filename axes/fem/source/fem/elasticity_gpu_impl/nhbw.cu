#include "../elasticity_gpu.cuh"
#include "ax/fem/elasticity/neohookean_bw.hpp"

namespace ax::fem {
template class ElasticityCompute_GPU<2, elasticity::NeoHookeanBW>;
template class ElasticityCompute_GPU<3, elasticity::NeoHookeanBW>;
}  // namespace ax::fem