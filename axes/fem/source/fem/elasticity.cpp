#include "ax/fem/elasticity.hpp"

namespace ax::fem {

template class ElasticityComputeBase<2>;
template class ElasticityComputeBase<3>;

}  // namespace ax::fem