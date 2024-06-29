#include "ax/fem/laplace_matrix.hpp"

#include "ax/fem/elements/p1.hpp"

namespace ax::fem {

template <idx dim> math::spmatr LaplaceMatrixCompute<dim>::operator()(real W) {
  math::sp_coeff_list l_coef;
  l_coef.reserve(mesh_.GetNumElements() * (dim + 1) * (dim + 1));
  idx nE = mesh_.GetNumElements();
  for (idx i = 0; i < nE; ++i) {
    math::veci<dim + 1> elem = mesh_.GetElement(i);
    // Should be faster than you integrate the Partial Partial...
    math::matr<dim + 1, dim + 1> C;
    for (idx i = 0; i <= dim; ++i) {
      math::vecr<dim> v = mesh_.GetVertex(elem[i]);
      for (idx j = 0; j < dim; ++j) {
        C(i, j) = v[j];
      }
      C(i, dim) = 1;
    }

    constexpr real volume_of_unit_simplex = dim == 2 ? 0.5 : 1.0 / 6.0;
    real const volume = abs(C.determinant()) * volume_of_unit_simplex;
    C = C.inverse();

    math::matr<dim + 1, dim + 1> L_dense_local;
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        real lap = 0.;
        for (idx D = 0; D < dim; ++D) {
          lap += C(D, i) * C(D, j);
        }
        L_dense_local(i, j) = lap * W * volume;
      }
    }

    // NOTE: make spsd enough.
    auto [eigen_vectors, eigen_values] = math::eig(L_dense_local);
    eigen_values = eigen_values.cwiseMax(0);
    L_dense_local = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    for (idx i = 0; i <= dim; ++i) {
      for (idx j = 0; j <= dim; ++j) {
        l_coef.push_back({elem[i], elem[j], L_dense_local(i, j)});
      }
    }
  }
  idx dofs = mesh_.GetNumVertices();
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template <idx dim> math::spmatr LaplaceMatrixCompute<dim>::operator()(math::field1r const& W) {
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
        l_coef.push_back({elem[i], elem[j], lap * W(iElem)});
      }
    }
  }
  idx dofs = mesh_.GetNumVertices();
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template class LaplaceMatrixCompute<2>;
template class LaplaceMatrixCompute<3>;

}  // namespace ax::fem
