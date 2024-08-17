#pragma once
#include "ax/math/functional.hpp"

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

  math::RealVector3 operator()(real value) const;

  math::RealField3 operator()(math::RealVectorX const &values) const;

private:
  real low_;
  real high_;
  bool periodic_;
  cmap& colormap_;
};

}  // namespace ax::gl