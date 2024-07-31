#include "../elasticity_gpu.cuh"
#include "ax/fem/elasticity/linear.hpp"

namespace ax::fem {
template class ElasticityCompute_GPU<2, elasticity::Linear>;
template class ElasticityCompute_GPU<3, elasticity::Linear>;
}  // namespace ax::fem