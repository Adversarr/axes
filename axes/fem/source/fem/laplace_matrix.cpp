#include "ax/fem/laplace_matrix.hpp"

#include "ax/fem/elements/p1.hpp"

namespace ax::fem {

template <idx dim> math::sp_matxxr LaplaceMatrixCompute<dim>::operator()(real W) {
  math::sp_coeff_list l_coef;
  l_coef.reserve(mesh_.GetNumElements() * (dim + 1) * (dim + 1));
  idx nE = mesh_.GetNumElements();
  for (idx i = 0; i < nE; ++i) {
    math::veci<dim + 1> elem = mesh_.GetElement(i);
    std::array<math::vecr<dim>, dim + 1> vert;
    for (idx i = 0; i <= dim; ++i) {
      vert[i] = mesh_.GetVertex(elem[i]);
    }
    elements::P1Element<dim> E(vert);
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        real lap = 0.;
        for (idx D = 0; D < dim; ++D) {
          lap += E.Integrate_PF_PF(i, j, D, D);
        }
        for (idx D = 0; D < dim; ++D) {
          l_coef.push_back({dim * elem[i] + D, dim * elem[j] + D, lap * W});
        }
      }
    }
  }
  idx dofs = mesh_.GetNumVertices() * dim;
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template <idx dim> math::sp_matxxr LaplaceMatrixCompute<dim>::operator()(math::field1r const& W) {
  math::sp_coeff_list l_coef;
  l_coef.reserve(mesh_.GetNumElements() * (dim + 1) * (dim + 1));
  idx nE = mesh_.GetNumElements();
  for (idx iElem = 0; iElem < nE; ++iElem) {
    math::veci<dim + 1> elem = mesh_.GetElement(iElem);
    std::array<math::vecr<dim>, dim + 1> vert;
    for (idx i = 0; i <= dim; ++i) {
      vert[i] = mesh_.GetVertex(elem[i]);
    }
    elements::P1Element<dim> E(vert);
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        real lap = 0.;
        for (idx D = 0; D < dim; ++D) {
          lap += E.Integrate_PF_PF(i, j, D, D);
        }
        for (idx D = 0; D < dim; ++D) {
          l_coef.push_back({dim * elem[i] + D, dim * elem[j] + D, lap * W(iElem)});
        }
      }
    }
  }
  idx dofs = mesh_.GetNumVertices() * dim;
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template class LaplaceMatrixCompute<2>;
template class LaplaceMatrixCompute<3>;

}  // namespace ax::fem
