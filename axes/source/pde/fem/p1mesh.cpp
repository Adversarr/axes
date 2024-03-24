#include "axes/pde/fem/p1mesh.hpp"

namespace ax::pde::fem {

template<idx dim>
geo::SurfaceMesh P1Mesh<dim>:: ExtractSurface() const {
  AX_CHECK(dim == 3) << "Only 3D P1Mesh is supported";
  geo::SurfaceMesh surface;
  auto const& elem = this->elements_;
  auto const& vert = this->vertices_;
  if constexpr (dim == 3) {
    surface.first = vert;
  }
  surface.second.resize(3, elem.cols() * 4);
  for (idx i = 0; i < elem.cols(); ++i){
    auto const& e = elem.col(i);
    surface.second.col(4 * i) = math::vec3i{e(0), e(1), e(2)};
    surface.second.col(4 * i + 1) = math::vec3i{e(0), e(1), e(3)};
    surface.second.col(4 * i + 2) = math::vec3i{e(1), e(2), e(3)};
    surface.second.col(4 * i + 3) = math::vec3i{e(2), e(0), e(3)};
  }
  return surface;
}

template class P1Mesh<2>;
template class P1Mesh<3>;

}