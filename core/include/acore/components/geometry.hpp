#pragma once
#include "./common.hpp"  // IWYU pragma: export
#include "acore/math/field.hpp"
namespace axes::components {

class SimplicalGeometry {
  Field<RealVector3> vertices_;
  Field<RealVector3> normals_;
  Field<IndexVector3> triangles_;
  Field<IndexVector2> segments_;
  Field<Index> points_;
};

}  // namespace axes::components
