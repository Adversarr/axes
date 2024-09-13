#include "ax/fem/laplace_matrix.hpp"

#include "ax/fem/elements/p1.hpp"

namespace ax::fem {

template <int dim> math::RealSparseMatrix LaplaceMatrixCompute<dim>::operator()(Real W) {
  math::RealSparseCOO l_coef;
  auto const total = static_cast<size_t>(mesh_.GetNumElements() * (dim + 1) * (dim + 1));
  l_coef.reserve(total);
  Index nE = mesh_.GetNumElements();
  for (Index i = 0; i < nE; ++i) {
    math::IndexVector<dim + 1> elem = mesh_.GetElement(i);
    // Should be faster than you integrate the Partial Partial...
    math::RealMatrix<dim + 1, dim + 1> C;
    for (Index i = 0; i <= dim; ++i) {
      math::RealVector<dim> v = mesh_.GetVertex(elem[i]);
      for (Index j = 0; j < dim; ++j) {
        C(i, j) = v[j];
      }
      C(i, dim) = 1;
    }

    constexpr Real volume_of_unit_simplex = dim == 2 ? 0.5 : 1.0 / 6.0;
    Real const volume = abs(C.determinant()) * volume_of_unit_simplex;
    C = C.inverse().eval();

    math::RealMatrix<dim + 1, dim + 1> L_dense_local;
    for (Index i = 0; i <= dim; ++i) {
      for (Index j = 0; j <= dim; ++j) {
        Real lap = 0.;
        for (Index D = 0; D < dim; ++D) {
          lap += C(D, i) * C(D, j);
        }
        L_dense_local(i, j) = lap * volume;
      }
    }

    // NOTE: make spsd enough.
    auto [eigen_vectors, eigen_values] = math::eig(L_dense_local);
    eigen_values = eigen_values.cwiseMax(0);
    L_dense_local = W * eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    for (Index i = 0; i <= dim; ++i) {
      for (Index j = 0; j <= dim; ++j) {
        l_coef.push_back({elem[i], elem[j], L_dense_local(i, j)});
      }
    }
  }
  Index dofs = mesh_.GetNumVertices();
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template <int dim> math::RealSparseMatrix LaplaceMatrixCompute<dim>::operator()(math::RealField1 const& W) {
  math::RealSparseCOO l_coef;
  auto const total = static_cast<size_t>(mesh_.GetNumElements() * (dim + 1) * (dim + 1));
  l_coef.reserve(total);
  Index nE = mesh_.GetNumElements();
  for (Index iElem = 0; iElem < nE; ++iElem) {
    math::IndexVector<dim + 1> elem = mesh_.GetElement(iElem);
    std::array<math::RealVector<dim>, dim + 1> vert;
    for (Index i = 0; i <= dim; ++i) {
      vert[static_cast<size_t>(i)] = mesh_.GetVertex(elem[i]);
    }
    elements::P1Element<dim> E(vert);
    math::RealMatrix<dim + 1, dim + 1> L_dense_local;
    for (Index i = 0; i <= dim; ++i) {
      for (Index j = 0; j <= dim; ++j) {
        Real lap = 0.;
        for (Index D = 0; D < dim; ++D) {
          lap += E.Integrate_PF_PF(i, j, D, D);
        }
        L_dense_local(i, j) = lap;
      }
    }

    // NOTE: make spsd enough.
    auto [eigen_vectors, eigen_values] = math::eig(L_dense_local);
    eigen_values = eigen_values.cwiseMax(0);
    L_dense_local = W(iElem) * eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    for (Index i = 0; i <= dim; ++i) {
      for (Index j = 0; j <= dim; ++j) {
        l_coef.push_back({elem[i], elem[j], L_dense_local(i, j)});
      }
    }
  }
  Index dofs = mesh_.GetNumVertices();
  return math::make_sparse_matrix(dofs, dofs, l_coef);
}

template class LaplaceMatrixCompute<2>;
template class LaplaceMatrixCompute<3>;

}  // namespace ax::fem
