#pragma once
#include "ax/geometry/intersection/common.hpp"
namespace ax::geo {


  
// Discrete time version
template <idx dim> AX_HOST_DEVICE CollisionInfo detect_edge_edge(Segment<dim> const& a,
                                                                 Segment<dim> const& b,
                                                                 real tol,
                                                                 real dt = 0.0) {}

// Continuous time version
template <idx dim>
AX_HOST_DEVICE CollisionInfo detect_edge_edge(Segment<dim> const& a0, Segment<dim> const& a1,
                                              Segment<dim> const& b0, Segment<dim> const& b1, real tol) {}
}