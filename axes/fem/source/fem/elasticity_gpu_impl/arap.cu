#include "../elasticity_gpu.cuh"
#include "ax/fem/elasticity/arap.hpp"

namespace ax::fem {
template class ElasticityCompute_GPU<2, elasticity::IsotropicARAP>;
template class ElasticityCompute_GPU<3, elasticity::IsotropicARAP>;
}  // namespace ax::fem