#pragma once
#include "axes/optim/spsdm.hpp"
namespace ax::optim {

class EigenvalueModification : public SpsdModificationBase {
public:
  virtual StatusOr<math::matxxr> Modify(math::matxxr const& A,
                                        utils::Opt const& opt = {});

  
  real min_eigval_{1e-6};
};

}  // namespace ax::optim
