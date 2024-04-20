#include "ax/fem/laplace_matrix.hpp"
#include "ax/core/echo.hpp"
#include "ax/fem/mesh/p1mesh.hpp"
#include "ax/fem/elements/p1.hpp"

namespace ax::fem{

template<idx dim>
math::sp_matxxr p1_uniform(real w, P1Mesh<dim> const& mesh) {
  math::sp_coeff_list l_coef;
  l_coef.reserve(mesh.GetNumElements() * (dim + 1) * (dim + 1));
  idx nE = mesh.GetNumElements();
  for (idx i = 0; i < nE; ++i) {
    math::veci<dim + 1> elem = mesh.GetElement(i);
    std::array<math::vecr<dim>, dim + 1> vert;
    for (idx i = 0; i <= dim; ++i) {
      vert[i] = mesh.GetVertex(elem[i]);
    }
    elements::P1Element<dim> E(vert);
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        real lap = 0.;
        for (idx D = 0; D < dim; ++D) {
          lap += E.Integrate_PF_PF(i, j, D, D);
        }
        for (idx D = 0; D < dim; ++D) {
          l_coef.push_back({dim * elem[i] + D, dim * elem[j] + D, lap * w});
        }
      }
    }
  }
  idx dofs = mesh.GetNumVertices() * dim;
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template<idx dim>
math::sp_matxxr p1_per_element(math::field1r const& w, P1Mesh<dim> const& mesh) {
  math::sp_coeff_list l_coef;
  l_coef.reserve(mesh.GetNumElements() * (dim + 1) * (dim + 1));
  idx nE = mesh.GetNumElements();
  for (idx iElem = 0; iElem < nE; ++iElem) {
    math::veci<dim + 1> elem = mesh.GetElement(iElem);
    std::array<math::vecr<dim>, dim + 1> vert;
    for (idx i = 0; i <= dim; ++i) {
      vert[i] = mesh.GetVertex(elem[i]);
    }
    elements::P1Element<dim> E(vert);
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        real lap = 0.;
        for (idx D = 0; D < dim; ++D) {
          lap += E.Integrate_PF_PF(i, j, D, D);
        }
        for (idx D = 0; D < dim; ++D) {
          l_coef.push_back({dim * elem[i] + D, dim * elem[j] + D, lap * w(iElem)});
        }
      }
    }
  }
  idx dofs = mesh.GetNumVertices() * dim;
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template<idx dim>
math::sp_matxxr LaplaceMatrixCompute<dim>::operator()(real W){
  if (mesh_->GetType() == MeshType::kP1) {
    return p1_uniform(W, dynamic_cast<P1Mesh<dim> const&>(*mesh_));
  } else {
    AX_CHECK(false) << "Unsupported mesh type";
  }
  AX_UNREACHABLE();
}

template<idx dim>
math::sp_matxxr LaplaceMatrixCompute<dim>::operator()(math::field1r const& W){
  if (mesh_->GetType() == MeshType::kP1) {
    return p1_per_element(W, dynamic_cast<P1Mesh<dim> const&>(*mesh_));
  } else {
    AX_CHECK(false) << "Unsupported mesh type";
  }
  AX_UNREACHABLE();
}

template class LaplaceMatrixCompute<2>;
template class LaplaceMatrixCompute<3>;

}