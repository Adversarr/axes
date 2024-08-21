#pragma once
#include "ax/math/functional.hpp"

namespace ax::gl {

using cmap = Real const[256][3];

// This is taken from DiffFR.
extern Real const colormap_bwr[256][3];
extern Real const colormap_coolwarm[256][3];
extern Real const colormap_jet[256][3];
extern Real const colormap_plasma[256][3];
extern Real const colormap_seismic[256][3];

class Colormap {
public:
  Colormap(Real low, Real high, bool periodic = false, cmap& colormap = colormap_jet)
      : low_(low), high_(high), periodic_(periodic), colormap_(colormap) {}

  math::RealVector3 operator()(Real value) const;

  math::RealField3 operator()(math::RealVectorX const &values) const;

private:
  Real low_;
  Real high_;
  bool periodic_;
  cmap& colormap_;
};

}  // namespace ax::gl