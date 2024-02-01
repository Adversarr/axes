#pragma once

#include <axes/math/common.hpp>
namespace ax::gui {

struct Points3D {
  math::field3r vertices_;
  math::field3r colors_;
};

}  // namespace ax::gui
