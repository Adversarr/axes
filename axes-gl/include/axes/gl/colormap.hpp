#pragma once
#include "axes/math/functional.hpp"

namespace ax::gl {

using cmap = real const[256][3];

// This is taken from DiffFR.
extern real const colormap_bwr[256][3];
extern real const colormap_coolwarm[256][3];
extern real const colormap_jet[256][3];
extern real const colormap_plasma[256][3];
extern real const colormap_seismic[256][3];

class Colormap {
public:
  Colormap(real low, real high, bool periodic = false, cmap& colormap = colormap_jet)
      : low_(low), high_(high), periodic_(periodic), colormap_(colormap) {}

  math::vec3r operator()(real value) const;

  math::field3r operator()(math::vecxr const &values) const;

private:
  real low_;
  real high_;
  bool periodic_;
  cmap& colormap_;
};

}  // namespace ax::gl