#pragma once
#include "axes/math/common.hpp"
namespace ax::gui {

struct Lines {
  math::field3r vertices_;
  math::field3r colors_;
  math::field2i indices_;
};


}  // namespace ax::gui
