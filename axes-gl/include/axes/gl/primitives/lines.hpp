#pragma once
#include "axes/math/common.hpp"
namespace ax::gl {

class Lines {
public:
  math::field3r vertices_;
  math::field4r colors_;
  math::field2i indices_;

  bool flush_{false};
};

}  // namespace ax::gl
