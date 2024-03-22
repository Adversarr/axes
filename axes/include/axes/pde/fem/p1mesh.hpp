#pragma once
#include "axes/pde/elements/p1.hpp"
#include "mesh.hpp"

namespace ax::pde::fem {

template<idx dim>
class P1Mesh: public MeshBase<dim> {
public:
  using element_t = MeshBase<dim>::element_t;
  using vertex_t = MeshBase<dim>::vertex_t;
  using element_list_t = MeshBase<dim>::element_list_t;
  using vertex_list_t = MeshBase<dim>::vertex_list_t;
  using boundary_value_t = MeshBase<dim>::boundary_value_t;
  using boundary_value_list_t = MeshBase<dim>::boundary_value_list_t;

  // Constructors
  P1Mesh() : MeshBase<dim>(MeshType::kP1) {}
  AX_DECLARE_CONSTRUCTOR(P1Mesh, default, default);
  virtual ~P1Mesh() = default;

  // Accessors
  idx GetNumVerticesPerElement() const noexcept final { return dim + 1;}

  // Helpers
  geo::SurfaceMesh ExtractSurface() const override;

  using element_impl_t = elements::P1Element<dim>;
};

}