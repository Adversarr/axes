/**
 * @brief Incremental Potential Contact Implementation.
 * @date 2024-03-28
 * 
 */

#pragma once
#include "axes/math/common.hpp"
namespace ax::pde::ip {

// NOTE: This may not be able to represent all the simulated objects, but currently
//       this design is good enough.
AX_DECLARE_ENUM(PrimitiveKind) {
  kPlane,           ///< Indicates a plane, e.g. a ground plane, wall, with normal, and the width is infinite
  kSurfaceVertex,   ///< Indicates a vertex in parameterized surface, e.g. TetMesh, SurfaceMesh
  kSurfaceEdge,     ///< Indicates an edge  in parameterized surface, e.g. TetMesh, SurfaceMesh
  kSurfaceFace,     ///< Indicates a face in parameterized surface, e.g. TetMesh, SurfaceMesh, Invalid in 2D apps.
  kParticle         ///< Indicates a point particle, with relatively small radius
};

struct Primitive {
  PrimitiveKind kind_;
  idx object_id_;
  idx prim_id_on_object_;
};

template<idx dim>
struct ContactInfo {
  Primitive prim1_;
  Primitive prim2_;
  real toi_;
};

}